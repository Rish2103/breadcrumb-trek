#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <deque>
#include <cassert>

#include "ahrs/NavigationEngine.hpp"
#include "ahrs/MathTypes.hpp"
#include "ahrs/MagneticReliability.hpp"
#include "ahrs/StationaryDetector.hpp"
#include "pdr/StepDetector.hpp"
#include "pdr/StrideEstimator.hpp"
#include "ConstrainedCourseObserver.hpp"

#define ASSERT_TEST(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            assert(false); \
        } else { \
            std::cout << "  PASS: " << msg << "\n"; \
        } \
    } while(0)

struct SensorRow {
    int64_t timestampNs;
    int sensorType;
    float x, y, z;
};

std::vector<SensorRow> loadCsv(const std::string& path) {
    std::vector<SensorRow> rows;
    std::ifstream file(path);
    if (!file.is_open()) return rows;
    std::string line;
    if (!std::getline(file, line)) return rows;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string item;
        SensorRow row;

        if (!std::getline(ss, item, ',')) continue;
        row.timestampNs = std::stoll(item);

        if (!std::getline(ss, item, ',')) continue;
        row.sensorType = std::stoi(item);

        if (!std::getline(ss, item, ',')) continue;
        row.x = std::stof(item);

        if (!std::getline(ss, item, ',')) continue;
        row.y = std::stof(item);

        if (!std::getline(ss, item, ',')) continue;
        row.z = std::stof(item);

        rows.push_back(row);
    }
    return rows;
}

// ============================================================================
// PART 1: DETERMINISTIC SYNTHETIC TESTS (Requirement 11)
// ============================================================================
void runSyntheticTests() {
    std::cout << "=======================================================\n";
    std::cout << ">>> RUNNING M3.6 SYNTHETIC VALIDATION SUITE (Req 11)\n";
    std::cout << "=======================================================\n";
    constexpr double deg2rad = M_PI / 180.0;
    constexpr double rad2deg = 180.0 / M_PI;

    // 1. Straight North walking
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true); // Start facing North
        std::vector<std::pair<double, double>> acc;
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            acc.push_back({0.1 * std::sin(4 * t), 2.5 * std::sin(2 * M_PI * 1.8 * t)});
        }
        float course = obs.update(acc, 0.0f, false);
        ASSERT_TEST(std::abs(course * rad2deg) < 2.0, "Test 1: Straight North walking maintains course ~0°");
    }

    // 2. Straight East walking
    {
        ConstrainedCourseObserver obs;
        obs.reset(static_cast<float>(90.0 * deg2rad), true); // Start facing East
        std::vector<std::pair<double, double>> acc;
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            acc.push_back({2.5 * std::sin(2 * M_PI * 1.8 * t), 0.1 * std::sin(4 * t)});
        }
        float course = obs.update(acc, static_cast<float>(90.0 * deg2rad), false);
        ASSERT_TEST(std::abs(course * rad2deg - 90.0) < 2.0, "Test 2: Straight East walking maintains course ~90°");
    }

    // 3. Arbitrary phone yaw (phone held at 45° yaw while walking North)
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true); // True course is North
        std::vector<std::pair<double, double>> acc;
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            acc.push_back({0.1 * std::sin(4 * t), 2.5 * std::sin(2 * M_PI * 1.8 * t)});
        }
        // Phone bearing is offset at 45°
        float course = obs.update(acc, static_cast<float>(45.0 * deg2rad), false);
        ASSERT_TEST(std::abs(course * rad2deg) < 5.0, "Test 3: Arbitrary static phone yaw (45°) rejected; course remains North (<5°)");
    }

    // 4. Phone yaw oscillation while travel direction remains constant
    // (Simulate 10 walking steps where phone swings +-60° with arm swing)
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true); // Walking North
        float maxCourseDeviation = 0.0f;
        for (int step = 0; step < 10; ++step) {
            std::vector<std::pair<double, double>> acc;
            for (int i = 0; i < 50; ++i) {
                double t = i * 0.02;
                acc.push_back({0.2 * std::sin(4 * t), 2.5 * std::sin(2 * M_PI * 1.8 * t)});
            }
            float phoneSwing = ((step % 2 == 0) ? +60.0f : -60.0f) * static_cast<float>(deg2rad);
            float course = obs.update(acc, phoneSwing, false);
            maxCourseDeviation = std::max(maxCourseDeviation, std::abs(course * static_cast<float>(rad2deg)));
        }
        std::cout << "  Max course deviation during +-60° phone swing: " << maxCourseDeviation << "°\n";
        ASSERT_TEST(maxCourseDeviation < 6.0f, "Test 4: +-60° phone yaw oscillation suppressed (travel course error < 6°)");
    }

    // 5. PCA +-180° ambiguity resolution
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true); // Walking North
        std::vector<std::pair<double, double>> acc;
        // Generate pure North-South oscillation where PCA axis has 180° ambiguity
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            acc.push_back({0.0, 2.5 * std::sin(2 * M_PI * 1.8 * t)});
        }
        obs.update(acc, 0.0f, false);
        ASSERT_TEST(std::abs(obs.getLastPca().orientedAngleRad * rad2deg) < 5.0,
                    "Test 5: PCA 180° ambiguity resolved to forward North (oriented angle ~0°, NOT 180°)");
        ASSERT_TEST(obs.getSignFlips180() == 0, "Test 5: Zero 180° sign flips occurred");
    }

    // 6. Low eigenvalue separation (circular noise rejection)
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true);
        std::vector<std::pair<double, double>> circularAcc;
        // Exact circular motion over full period: lambda1 == lambda2, confidence = 0
        for (int i = 0; i < 50; ++i) {
            double angle = 2.0 * M_PI * i / 50.0;
            circularAcc.push_back({2.0 * std::cos(angle), 2.0 * std::sin(angle)});
        }
        obs.update(circularAcc, 0.0f, false);
        ASSERT_TEST(!obs.getLastPca().accepted, "Test 6: Low eigenvalue separation correctly REJECTS PCA update");
        ASSERT_TEST(obs.getRejectedPcaUpdates() == 1, "Test 6: Rejected count incremented");
    }

    // 7. Genuine 90° travel turn
    // (Turn from North to East over 4 steps)
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true);
        // Step 1: North
        std::vector<std::pair<double, double>> accN;
        for (int i = 0; i < 50; ++i) accN.push_back({0.0, 2.5 * std::sin(i * 0.2)});
        obs.update(accN, 0.0f, false);

        // Turn to East: acceleration shifts to East and phone turns to East
        std::vector<std::pair<double, double>> accE;
        for (int i = 0; i < 50; ++i) accE.push_back({2.5 * std::sin(i * 0.2), 0.0});

        float finalCourse = 0.0f;
        for (int step = 0; step < 6; ++step) {
            finalCourse = obs.update(accE, static_cast<float>(90.0 * deg2rad), false);
        }
        std::cout << "  Course after 90° turn: " << (finalCourse * rad2deg) << "°\n";
        ASSERT_TEST(std::abs(finalCourse * rad2deg - 90.0) < 15.0,
                    "Test 7: Genuine 90° turn successfully followed by observer");
    }

    // 8. Genuine 180° travel reversal (walk North, turn around, walk South)
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true); // North
        std::vector<std::pair<double, double>> accN;
        for (int i = 0; i < 50; ++i) accN.push_back({0.0, 2.5 * std::sin(i * 0.2)});
        obs.update(accN, 0.0f, false);

        // 180° turnaround hint provided (stationary turn occurred)
        float reversedCourse = obs.update(accN, static_cast<float>(180.0 * deg2rad), false, true);
        ASSERT_TEST(std::abs(std::abs(reversedCourse * rad2deg) - 180.0) < 1.0,
                    "Test 8: Stationary 180° turnaround hint reverses course cleanly to ~±180°");
    }

    // 9. Stationary pause followed by phone rotation (User stops and looks around)
    {
        ConstrainedCourseObserver obs;
        obs.reset(0.0f, true); // Walking North
        std::vector<std::pair<double, double>> acc;
        for (int i = 0; i < 50; ++i) acc.push_back({0.0, 2.5 * std::sin(i * 0.2)});
        obs.update(acc, 0.0f, false); // Active step

        // Stop: isStationary = true, user rotates phone to 90° East while stopped
        float stoppedCourse = obs.update({}, static_cast<float>(90.0 * deg2rad), true);
        ASSERT_TEST(std::abs(stoppedCourse * rad2deg) < 1.0,
                    "Test 9: Stationary period preserves travel course (course does NOT rotate with phone)");
    }

    std::cout << ">>> ALL 9 SYNTHETIC TESTS PASSED SUCCESSFULLY!\n";
}

// ============================================================================
// PART 2: PHYSICAL SENSOR REPLAY (Req 12, 13, 14)
// ============================================================================
struct ReplayMetrics {
    uint32_t stepCount = 0;
    float cumulativeDistance = 0.0f;
    float finalEast = 0.0f;
    float finalNorth = 0.0f;
    float stepHeadingVar = 0.0f;
    float straightHeadingStdDev = 0.0f;
    int jumpsOver30 = 0;
    int jumpsOver60 = 0;
    int signFlips180 = 0;
    float meanPcaConfidence = 0.0f;
    uint32_t acceptedPcaCount = 0;
    uint32_t rejectedPcaCount = 0;
};

// Mode: 0 = M3.1 Baseline (Circular mean phone heading)
//       1 = M3.5 Candidate A (2-step bilateral cycle)
//       2 = M3.5 Candidate B (Raw Earth-frame PCA)
//       3 = M3.6 Constrained Travel-Course Observer
ReplayMetrics runReplayVariant(const std::vector<SensorRow>& rows, int mode) {
    Madgwick9DOF ahrs;
    MagneticReliability magRel;
    StationaryDetector statDet;
    StepDetector stepDetector;
    StrideEstimator strideEstimator;
    ConstrainedCourseObserver courseObs;

    struct TimestampedHeading {
        int64_t tNs;
        float headingRad;
    };
    std::deque<TimestampedHeading> headingHistory;

    struct EarthAccelSample {
        int64_t tNs;
        double aE;
        double aN;
    };
    std::deque<EarthAccelSample> earthAccelHistory;

    int64_t lastStepTimeNs = 0;
    int64_t prevStepTimeNs = 0;
    float east = 0.0f, north = 0.0f, cumulativeDist = 0.0f;
    uint32_t stepCount = 0;

    int64_t lastAccelT = 0, lastGyroT = 0, lastMagT = 0;
    Vec3 latestAccel{0,0,0}, latestMag{0,0,0};
    bool init = false;

    std::vector<float> stepHeadingsDeg;
    float prevTravelHeading = 0.0f;
    bool hasPrevTravelHeading = false;
    int jumpsOver30 = 0, jumpsOver60 = 0, signFlips180 = 0;

    double sumPcaConf = 0.0;
    int pcaCount = 0;

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    constexpr float rad2deg = static_cast<float>(180.0 / M_PI);

    for (const auto& r : rows) {
        if (r.sensorType == 1) { // ACCEL
            latestAccel = {r.x, r.y, r.z};
            lastAccelT = r.timestampNs;

            // Earth-frame horizontal acceleration
            Vec3 aEarth = rotateBodyToEarth(ahrs.q, latestAccel);
            double aEast = -aEarth.y;
            double aNorth = aEarth.x;

            while (!earthAccelHistory.empty() && (r.timestampNs - earthAccelHistory.front().tNs) > 2500000000LL) {
                earthAccelHistory.pop_front();
            }
            earthAccelHistory.push_back({r.timestampNs, aEast, aNorth});

            float curHeading = static_cast<float>(phoneForwardBearing(ahrs.q));
            while (!headingHistory.empty() && (r.timestampNs - headingHistory.front().tNs) > 2500000000LL) {
                headingHistory.pop_front();
            }
            headingHistory.push_back({r.timestampNs, curHeading});

            float aMag = std::sqrt(r.x*r.x + r.y*r.y + r.z*r.z);
            bool isStat = statDet.isStationary();
            StepDetector::Step step = stepDetector.push(aMag, r.timestampNs, isStat);

            if (step.detected) {
                stepCount++;
                float stride = strideEstimator.estimate(step.aMax, step.aMin);
                cumulativeDist += stride;

                float travelHeadingRad = curHeading;

                if (mode == 0) {
                    // M3.1 BASELINE
                    int64_t startT = (lastStepTimeNs > 0 && (step.tNs - lastStepTimeNs) <= 2000000000LL)
                                     ? lastStepTimeNs : (step.tNs - 500000000LL);
                    double sinSum = 0.0, cosSum = 0.0;
                    int cnt = 0;
                    for (const auto& h : headingHistory) {
                        if (h.tNs >= startT && h.tNs <= step.tNs) {
                            sinSum += std::sin(h.headingRad);
                            cosSum += std::cos(h.headingRad);
                            cnt++;
                        }
                    }
                    if (cnt > 0) travelHeadingRad = static_cast<float>(std::atan2(sinSum / cnt, cosSum / cnt));
                } else if (mode == 1) {
                    // M3.5 CANDIDATE A (2-step cycle)
                    int64_t startT = (prevStepTimeNs > 0 && (step.tNs - prevStepTimeNs) <= 3000000000LL)
                                     ? prevStepTimeNs
                                     : (lastStepTimeNs > 0 ? lastStepTimeNs : (step.tNs - 1000000000LL));
                    double sinSum = 0.0, cosSum = 0.0;
                    int cnt = 0;
                    for (const auto& h : headingHistory) {
                        if (h.tNs >= startT && h.tNs <= step.tNs) {
                            sinSum += std::sin(h.headingRad);
                            cosSum += std::cos(h.headingRad);
                            cnt++;
                        }
                    }
                    if (cnt > 0) travelHeadingRad = static_cast<float>(std::atan2(sinSum / cnt, cosSum / cnt));
                } else if (mode == 2) {
                    // M3.5 CANDIDATE B (Raw PCA)
                    int64_t startT = (lastStepTimeNs > 0 && (step.tNs - lastStepTimeNs) <= 2000000000LL)
                                     ? lastStepTimeNs : (step.tNs - 500000000LL);
                    std::vector<std::pair<double, double>> winAcc;
                    for (const auto& a : earthAccelHistory) {
                        if (a.tNs >= startT && a.tNs <= step.tNs) winAcc.push_back({a.aE, a.aN});
                    }
                    // Compute PCA manually for Candidate B
                    if (winAcc.size() >= 5) {
                        double meanE = 0.0, meanN = 0.0;
                        for (const auto& s : winAcc) { meanE += s.first; meanN += s.second; }
                        meanE /= winAcc.size(); meanN /= winAcc.size();
                        double c11 = 0.0, c12 = 0.0, c22 = 0.0;
                        for (const auto& s : winAcc) {
                            c11 += (s.first - meanE)*(s.first - meanE);
                            c12 += (s.first - meanE)*(s.second - meanN);
                            c22 += (s.second - meanN)*(s.second - meanN);
                        }
                        double T = c11 + c22, D = c11*c22 - c12*c12;
                        double lambda1 = T*0.5 + std::sqrt(std::max(0.0, (T*0.5)*(T*0.5) - D));
                        double vE = (std::abs(c12) > 1e-6) ? (lambda1 - c22) : (c11 >= c22 ? 1.0 : 0.0);
                        double vN = (std::abs(c12) > 1e-6) ? c12 : (c11 >= c22 ? 0.0 : 1.0);
                        double norm = std::hypot(vE, vN);
                        if (norm > 1e-12) {
                            vE /= norm; vN /= norm;
                            // Candidate B sign resolution via phone forward hemisphere
                            double dot = vE * std::sin(curHeading) + vN * std::cos(curHeading);
                            if (dot < 0.0) { vE = -vE; vN = -vN; }
                            travelHeadingRad = static_cast<float>(std::atan2(vE, vN));
                        }
                    }
                } else if (mode == 3) {
                    // M3.6 CONSTRAINED TRAVEL-COURSE OBSERVER
                    int64_t startT = (lastStepTimeNs > 0 && (step.tNs - lastStepTimeNs) <= 2000000000LL)
                                     ? lastStepTimeNs : (step.tNs - 500000000LL);
                    std::vector<std::pair<double, double>> winAcc;
                    for (const auto& a : earthAccelHistory) {
                        if (a.tNs >= startT && a.tNs <= step.tNs) winAcc.push_back({a.aE, a.aN});
                    }

                    // Average phone bearing over step
                    double sinSum = 0.0, cosSum = 0.0;
                    int cnt = 0;
                    for (const auto& h : headingHistory) {
                        if (h.tNs >= startT && h.tNs <= step.tNs) {
                            sinSum += std::sin(h.headingRad);
                            cosSum += std::cos(h.headingRad);
                            cnt++;
                        }
                    }
                    float stepPhoneHeading = (cnt > 0) ? static_cast<float>(std::atan2(sinSum / cnt, cosSum / cnt)) : curHeading;

                    travelHeadingRad = courseObs.update(winAcc, stepPhoneHeading, isStat);
                    sumPcaConf += courseObs.getLastPca().confidence;
                    pcaCount++;
                }

                // Displacements
                float dE = stride * std::sin(travelHeadingRad);
                float dN = stride * std::cos(travelHeadingRad);
                east += dE;
                north += dN;

                float hDeg = travelHeadingRad * rad2deg;
                stepHeadingsDeg.push_back(hDeg);

                if (mode == 3 && (stepCount <= 10 || (stepCount >= 40 && stepCount <= 50))) {
                    std::cout << "Step " << std::setw(2) << stepCount
                              << " | Course: " << std::setw(6) << std::fixed << std::setprecision(1) << hDeg << "°"
                              << " | Phone: " << std::setw(6) << (curHeading * rad2deg) << "°"
                              << " | PCA: " << std::setw(6) << (courseObs.getLastPca().axisAngleRad * rad2deg) << "°"
                              << " | Conf: " << std::setprecision(2) << courseObs.getLastPca().confidence
                              << " | Acc: " << (courseObs.getLastPca().accepted ? "YES" : "NO")
                              << " | CumE: " << std::setprecision(2) << east
                              << " | CumN: " << north
                              << "\n";
                }

                if (hasPrevTravelHeading) {
                    float delta = std::abs(static_cast<float>(wrapPi(travelHeadingRad - prevTravelHeading)));
                    if (delta > 30.0f * deg2rad) jumpsOver30++;
                    if (delta > 60.0f * deg2rad) jumpsOver60++;
                    if (std::abs(delta - static_cast<float>(M_PI)) < 25.0f * deg2rad) signFlips180++;
                }
                prevTravelHeading = travelHeadingRad;
                hasPrevTravelHeading = true;

                prevStepTimeNs = lastStepTimeNs;
                lastStepTimeNs = step.tNs;
            }
        } else if (r.sensorType == 4) { // GYRO
            if (!init) {
                init = true;
                lastGyroT = r.timestampNs;
                continue;
            }
            double dt = (r.timestampNs - lastGyroT) * 1e-9;
            lastGyroT = r.timestampNs;
            if (dt <= 0.0 || dt > 0.25) continue;

            bool accelFresh = (r.timestampNs - lastAccelT) < 100000000LL;
            bool magFresh = (r.timestampNs - lastMagT) < 300000000LL;
            Vec3 a = accelFresh ? latestAccel : Vec3{0,0,0};
            Vec3 m = magFresh ? latestMag : Vec3{0,0,0};
            float Cmag = magFresh ? magRel.getCmag() : 0.0f;

            statDet.update(a.x, a.y, a.z, r.x, r.y, r.z);
            Vec3 bias = statDet.gyroBias;
            float gcx = r.x - bias.x;
            float gcy = r.y - bias.y;
            float gcz = r.z - bias.z;

            ahrs.update(gcx, gcy, gcz, a.x, a.y, a.z, m.x, m.y, m.z, Cmag, static_cast<float>(dt));
        } else if (r.sensorType == 2) { // MAG
            latestMag = {r.x, r.y, r.z};
            lastMagT = r.timestampNs;
            magRel.update(r.x, r.y, r.z, ahrs.q, statDet.isStationary());
        }
    }

    ReplayMetrics m;
    m.stepCount = stepCount;
    m.cumulativeDistance = cumulativeDist;
    m.finalEast = east;
    m.finalNorth = north;
    m.jumpsOver30 = jumpsOver30;
    m.jumpsOver60 = jumpsOver60;
    m.signFlips180 = (mode == 3) ? courseObs.getSignFlips180() : signFlips180;
    m.meanPcaConfidence = (pcaCount > 0) ? static_cast<float>(sumPcaConf / pcaCount) : 0.0f;
    m.acceptedPcaCount = courseObs.getAcceptedPcaUpdates();
    m.rejectedPcaCount = courseObs.getRejectedPcaUpdates();

    // Step heading variation
    float varSum = 0.0f;
    for (size_t i = 1; i < stepHeadingsDeg.size(); ++i) {
        float r1 = stepHeadingsDeg[i - 1] * deg2rad;
        float r2 = stepHeadingsDeg[i] * deg2rad;
        varSum += std::abs(static_cast<float>(wrapPi(r2 - r1)) * rad2deg);
    }
    m.stepHeadingVar = stepHeadingsDeg.size() > 1 ? (varSum / (stepHeadingsDeg.size() - 1)) : 0.0f;

    // Straight walking stddev (steps 15 to 45)
    if (stepHeadingsDeg.size() > 15) {
        size_t sStart = 15;
        size_t sEnd = std::min(size_t(45), stepHeadingsDeg.size());
        float meanH = 0.0f;
        for (size_t i = sStart; i < sEnd; ++i) meanH += stepHeadingsDeg[i];
        meanH /= (sEnd - sStart);
        float varH = 0.0f;
        for (size_t i = sStart; i < sEnd; ++i) {
            float d = stepHeadingsDeg[i] - meanH;
            varH += d * d;
        }
        m.straightHeadingStdDev = std::sqrt(varH / (sEnd - sStart));
    }

    return m;
}

int main(int argc, char** argv) {
    runSyntheticTests();

    std::string csvPath = "/data/local/tmp/poco_walk.csv";
    if (argc > 1) csvPath = argv[1];

    auto rows = loadCsv(csvPath);
    if (rows.empty()) {
        std::cerr << "Failed to open " << csvPath << std::endl;
        return 1;
    }

    std::cout << "\n=======================================================\n";
    std::cout << ">>> REPLAYING POCO RECORDING (4 ESTIMATOR COMPARISON)\n";
    std::cout << "=======================================================\n";

    ReplayMetrics m0 = runReplayVariant(rows, 0); // M3.1 Baseline
    ReplayMetrics mA = runReplayVariant(rows, 1); // M3.5 Candidate A (2-Step)
    ReplayMetrics mB = runReplayVariant(rows, 2); // M3.5 Candidate B (Raw PCA)
    ReplayMetrics m36 = runReplayVariant(rows, 3); // M3.6 Constrained Observer

    std::cout << "\n===========================================================================================\n";
    std::cout << ">>> M3.6 FULL EXPERIMENT BENCHMARK: 4-WAY COMPARISON (Req 13, 14)\n";
    std::cout << "===========================================================================================\n";
    std::cout << std::left << std::setw(26) << "Metric"
              << std::setw(17) << "M3.1 (Baseline)"
              << std::setw(17) << "M3.5 Cand A"
              << std::setw(17) << "M3.5 Cand B"
              << std::setw(17) << "M3.6 (Observer)" << "\n";
    std::cout << "-------------------------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(26) << "Step Count"
              << std::setw(17) << m0.stepCount
              << std::setw(17) << mA.stepCount
              << std::setw(17) << mB.stepCount
              << std::setw(17) << m36.stepCount << "\n";
    std::cout << std::left << std::setw(26) << "Distance (m)"
              << std::setw(17) << std::fixed << std::setprecision(2) << m0.cumulativeDistance
              << std::setw(17) << mA.cumulativeDistance
              << std::setw(17) << mB.cumulativeDistance
              << std::setw(17) << m36.cumulativeDistance << "\n";
    std::cout << std::left << std::setw(26) << "Final E (m)"
              << std::setw(17) << std::fixed << std::setprecision(3) << m0.finalEast
              << std::setw(17) << mA.finalEast
              << std::setw(17) << mB.finalEast
              << std::setw(17) << m36.finalEast << "\n";
    std::cout << std::left << std::setw(26) << "Final N (m)"
              << std::setw(17) << std::fixed << std::setprecision(3) << m0.finalNorth
              << std::setw(17) << mA.finalNorth
              << std::setw(17) << mB.finalNorth
              << std::setw(17) << m36.finalNorth << "\n";
    std::cout << std::left << std::setw(26) << "Lateral Drift |E| (m)"
              << std::setw(17) << std::fixed << std::setprecision(3) << std::abs(m0.finalEast)
              << std::setw(17) << std::abs(mA.finalEast)
              << std::setw(17) << std::abs(mB.finalEast)
              << std::setw(17) << std::abs(m36.finalEast) << "\n";
    std::cout << std::left << std::setw(26) << "Course Variation (deg)"
              << std::setw(17) << std::fixed << std::setprecision(1) << m0.stepHeadingVar
              << std::setw(17) << mA.stepHeadingVar
              << std::setw(17) << mB.stepHeadingVar
              << std::setw(17) << m36.stepHeadingVar << "\n";
    std::cout << std::left << std::setw(26) << "Straight Course StdDev"
              << std::setw(17) << std::fixed << std::setprecision(1) << m0.straightHeadingStdDev
              << std::setw(17) << mA.straightHeadingStdDev
              << std::setw(17) << mB.straightHeadingStdDev
              << std::setw(17) << m36.straightHeadingStdDev << "\n";
    std::cout << std::left << std::setw(26) << "Jumps > 30°"
              << std::setw(17) << m0.jumpsOver30
              << std::setw(17) << mA.jumpsOver30
              << std::setw(17) << mB.jumpsOver30
              << std::setw(17) << m36.jumpsOver30 << "\n";
    std::cout << std::left << std::setw(26) << "Jumps > 60°"
              << std::setw(17) << m0.jumpsOver60
              << std::setw(17) << mA.jumpsOver60
              << std::setw(17) << mB.jumpsOver60
              << std::setw(17) << m36.jumpsOver60 << "\n";
    std::cout << std::left << std::setw(26) << "180° Sign Flips"
              << std::setw(17) << m0.signFlips180
              << std::setw(17) << mA.signFlips180
              << std::setw(17) << mB.signFlips180
              << std::setw(17) << m36.signFlips180 << "\n";
    std::cout << "-------------------------------------------------------------------------------------------\n";
    std::cout << "\n>>> M3.6 PCA DIAGNOSTIC TELEMETRY:\n";
    std::cout << "  Mean PCA Confidence:      " << std::fixed << std::setprecision(3) << m36.meanPcaConfidence << "\n";
    std::cout << "  Accepted PCA Updates:     " << m36.acceptedPcaCount << " (" << std::setprecision(1) << (m36.acceptedPcaCount * 100.0 / m36.stepCount) << "%)\n";
    std::cout << "  Rejected PCA Updates:     " << m36.rejectedPcaCount << " (" << std::setprecision(1) << (m36.rejectedPcaCount * 100.0 / m36.stepCount) << "%)\n";

    return 0;
}
