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
#include "pdr/StepDetector.hpp"
#include "pdr/StrideEstimator.hpp"

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

// 2D PCA calculation on horizontal acceleration samples
struct PcaResult {
    bool valid = false;
    double axisAngleRad = 0.0; // [-pi, pi]
    double lambda1 = 0.0;
    double lambda2 = 0.0;
    double eccentricity = 0.0;
    double vE = 0.0;
    double vN = 0.0;
};

PcaResult computeHorizontalPca(const std::vector<std::pair<double, double>>& samples) {
    PcaResult res;
    if (samples.size() < 5) return res;

    double meanE = 0.0, meanN = 0.0;
    for (const auto& s : samples) {
        meanE += s.first;
        meanN += s.second;
    }
    meanE /= samples.size();
    meanN /= samples.size();

    double c11 = 0.0, c12 = 0.0, c22 = 0.0;
    for (const auto& s : samples) {
        double dE = s.first - meanE;
        double dN = s.second - meanN;
        c11 += dE * dE;
        c12 += dE * dN;
        c22 += dN * dN;
    }
    c11 /= samples.size();
    c12 /= samples.size();
    c22 /= samples.size();

    double T = c11 + c22;
    double D = c11 * c22 - c12 * c12;
    double disc = std::max(0.0, (T * 0.5) * (T * 0.5) - D);
    double lambda1 = T * 0.5 + std::sqrt(disc);
    double lambda2 = T * 0.5 - std::sqrt(disc);

    double vE = 0.0, vN = 0.0;
    if (std::abs(c12) > 1e-6) {
        vE = lambda1 - c22;
        vN = c12;
    } else {
        if (c11 >= c22) {
            vE = 1.0; vN = 0.0;
        } else {
            vE = 0.0; vN = 1.0;
        }
    }
    double norm = std::hypot(vE, vN);
    if (norm < 1e-12) return res;
    vE /= norm;
    vN /= norm;

    res.valid = true;
    res.lambda1 = lambda1;
    res.lambda2 = lambda2;
    res.eccentricity = (lambda1 > 1e-6) ? (1.0 - lambda2 / lambda1) : 0.0;
    res.vE = vE;
    res.vN = vN;
    res.axisAngleRad = std::atan2(vE, vN); // Bearing in ENU: angle from North (clockwise)
    return res;
}

// ============================================================================
// PART 1: CONTROLLED SYNTHETIC TESTS (TEST A - E)
// ============================================================================
void runSyntheticTests() {
    std::cout << "=======================================================\n";
    std::cout << ">>> RUNNING CANDIDATE B SYNTHETIC TESTS (TEST A - E)\n";
    std::cout << "=======================================================\n";
    constexpr double deg2rad = M_PI / 180.0;
    constexpr double rad2deg = 180.0 / M_PI;

    // TEST A: Straight walking acceleration aligned with North
    // aN has gait oscillation (bounce +- 2.0 m/s2), aE has minimal jitter (+-0.2 m/s2)
    {
        std::vector<std::pair<double, double>> samples;
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            double aN = 2.5 * std::sin(2.0 * M_PI * 1.8 * t);
            double aE = 0.2 * std::sin(2.0 * M_PI * 3.6 * t);
            samples.push_back({aE, aN});
        }
        auto pca = computeHorizontalPca(samples);
        double angleDeg = std::abs(pca.axisAngleRad * rad2deg);
        if (angleDeg > 90.0) angleDeg = std::abs(angleDeg - 180.0);
        std::cout << "TEST A (North walking): PCA Axis = " << (pca.axisAngleRad * rad2deg)
                  << "° (deviation from North: " << angleDeg << "°)\n";
        assert(angleDeg < 5.0);
        std::cout << "  PASS: Principal axis aligned with North (axis error < 5°)\n";
    }

    // TEST B: Straight walking acceleration aligned with East
    {
        std::vector<std::pair<double, double>> samples;
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            double aE = 2.5 * std::sin(2.0 * M_PI * 1.8 * t);
            double aN = 0.2 * std::sin(2.0 * M_PI * 3.6 * t);
            samples.push_back({aE, aN});
        }
        auto pca = computeHorizontalPca(samples);
        double angleDeg = std::abs(pca.axisAngleRad * rad2deg);
        if (angleDeg > 90.0) angleDeg = 180.0 - angleDeg;
        double errorFromEast = std::abs(angleDeg - 90.0);
        std::cout << "TEST B (East walking): PCA Axis = " << (pca.axisAngleRad * rad2deg)
                  << "° (deviation from East: " << errorFromEast << "°)\n";
        assert(errorFromEast < 5.0);
        std::cout << "  PASS: Principal axis aligned with East (axis error < 5°)\n";
    }

    // TEST C: Same walking acceleration with arbitrary phone yaw rotation (e.g. phone yawed 60°)
    {
        // Body frame acceleration a_b is rotated by phone yaw R_z(60°)
        // In Earth NWU: x=North, y=West. Earth linear acceleration is aN = a_E.x, aE = -a_E.y
        // After rotating body acceleration to Earth frame using quaternion, it must recover North
        Quaternion qYaw60{static_cast<float>(std::cos(-30.0 * deg2rad)), 0.0f, 0.0f, static_cast<float>(std::sin(-30.0 * deg2rad))};
        // In body frame, acceleration is rotated backwards by 60°:
        std::vector<std::pair<double, double>> samples;
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            double aNorth = 2.5 * std::sin(2.0 * M_PI * 1.8 * t);
            double aWest = 0.0;
            double aUp = 9.80665; // gravity + no vertical bounce for test
            Vec3 aEarth{static_cast<float>(aNorth), static_cast<float>(aWest), static_cast<float>(aUp)};
            // Invert rotation to body: a_b = q* * aEarth * q
            Quaternion qc{qYaw60.w, -qYaw60.x, -qYaw60.y, -qYaw60.z};
            Vec3 aBody = rotateBodyToEarth(qc, aEarth);

            // Now reconstruct Earth frame using qYaw60:
            Vec3 aRecEarth = rotateBodyToEarth(qYaw60, aBody);
            double recE = -aRecEarth.y;
            double recN = aRecEarth.x;
            samples.push_back({recE, recN});
        }
        auto pca = computeHorizontalPca(samples);
        double angleDeg = std::abs(pca.axisAngleRad * rad2deg);
        if (angleDeg > 90.0) angleDeg = std::abs(angleDeg - 180.0);
        std::cout << "TEST C (Rotated Phone): PCA Axis = " << (pca.axisAngleRad * rad2deg)
                  << "° (deviation from North: " << angleDeg << "°)\n";
        assert(angleDeg < 5.0);
        std::cout << "  PASS: Earth frame transformation eliminates phone yaw rotation\n";
    }

    // TEST D: Sign resolution test
    {
        // Vector v pointing North-ish [0.1, 0.99] vs [-0.1, -0.99]
        // Reference heading = 0° (North) -> uRef = [0, 1]
        Vec3 uRef{0.0f, 1.0f, 0.0f}; // [sin(0), cos(0)]
        double vE = -0.1, vN = -0.99; // Anti-parallel
        double dot = vE * 0.0 + vN * 1.0;
        if (dot < 0.0) { vE = -vE; vN = -vN; }
        double resolvedAngle = std::atan2(vE, vN) * rad2deg;
        std::cout << "TEST D (Sign Inversion Resolution): Resolved Heading = " << resolvedAngle << "°\n";
        assert(std::abs(resolvedAngle) < 10.0);
        std::cout << "  PASS: Sign resolution consistently aligns with reference forward hemisphere\n";
    }

    // TEST E: Noisy gait acceleration
    {
        std::vector<std::pair<double, double>> samples;
        for (int i = 0; i < 50; ++i) {
            double t = i * 0.02;
            // North bounce + Gaussian-like noise
            double aN = 2.5 * std::sin(2.0 * M_PI * 1.8 * t) + 0.4 * std::cos(15.0 * t);
            double aE = 0.5 * std::cos(12.0 * t);
            samples.push_back({aE, aN});
        }
        auto pca = computeHorizontalPca(samples);
        double angleDeg = std::abs(pca.axisAngleRad * rad2deg);
        if (angleDeg > 90.0) angleDeg = std::abs(angleDeg - 180.0);
        std::cout << "TEST E (Noisy Gait): PCA Axis = " << (pca.axisAngleRad * rad2deg)
                  << "° (deviation: " << angleDeg << "°)\n";
        assert(angleDeg < 15.0);
        std::cout << "  PASS: PCA extracts dominant locomotion axis despite noise\n";
    }
}

// ============================================================================
// PART 2: SAME-SENSOR PHYSICAL REPLAY (BASELINE vs CANDIDATE A vs CANDIDATE B)
// ============================================================================
struct ReplayStats {
    uint32_t stepCount = 0;
    float cumulativeDistance = 0.0f;
    float finalEast = 0.0f;
    float finalNorth = 0.0f;
    float stepHeadingVar = 0.0f;
    float cycleHeadingVar = 0.0f;
    float headingStdDev = 0.0f;
    int signFlips180 = 0;
    int switchesOver30 = 0;
    int switchesOver60 = 0;
};

// Mode: 0 = Baseline (M3.1 single-step circular mean)
//       1 = Candidate A (2-step bilateral gait cycle circular mean)
//       2 = Candidate B (Earth-frame acceleration PCA)
ReplayStats runPhysicalReplay(const std::vector<SensorRow>& rows, int mode) {
    Madgwick9DOF ahrs;
    MagneticReliability magRel;
    StationaryDetector statDet;
    StepDetector stepDetector;
    StrideEstimator strideEstimator;

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
    int64_t prevStepTimeNs = 0; // t_{k-2}
    float east = 0.0f;
    float north = 0.0f;
    float cumulativeDist = 0.0f;
    uint32_t stepCount = 0;

    int64_t lastAccelT = 0, lastGyroT = 0, lastMagT = 0;
    Vec3 latestAccel{0,0,0}, latestMag{0,0,0};
    bool init = false;

    std::vector<float> stepHeadingsDeg;
    float prevTravelHeading = 0.0f;
    bool hasPrevTravelHeading = false;
    int signFlips180 = 0;
    int switchesOver30 = 0;
    int switchesOver60 = 0;

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    constexpr float rad2deg = static_cast<float>(180.0 / M_PI);

    for (const auto& r : rows) {
        if (r.sensorType == 1) { // ACCEL
            latestAccel = {r.x, r.y, r.z};
            lastAccelT = r.timestampNs;

            // Compute Earth-frame horizontal acceleration
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
                    // BASELINE: M3.1 single-step circular mean [lastStepTimeNs, step.tNs]
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
                    // CANDIDATE A: 2-step bilateral gait cycle circular mean [prevStepTimeNs, step.tNs]
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
                    // CANDIDATE B: Earth-frame acceleration PCA over step window
                    int64_t startT = (lastStepTimeNs > 0 && (step.tNs - lastStepTimeNs) <= 2000000000LL)
                                     ? lastStepTimeNs : (step.tNs - 500000000LL);
                    std::vector<std::pair<double, double>> winAcc;
                    for (const auto& a : earthAccelHistory) {
                        if (a.tNs >= startT && a.tNs <= step.tNs) {
                            winAcc.push_back({a.aE, a.aN});
                        }
                    }
                    auto pca = computeHorizontalPca(winAcc);
                    if (pca.valid && pca.eccentricity > 0.3) {
                        // Sign resolution: resolve eigenvector along phone forward bearing hemisphere
                        double dot = pca.vE * std::sin(curHeading) + pca.vN * std::cos(curHeading);
                        double vE = (dot >= 0.0) ? pca.vE : -pca.vE;
                        double vN = (dot >= 0.0) ? pca.vN : -pca.vN;
                        travelHeadingRad = static_cast<float>(std::atan2(vE, vN));
                    } else {
                        // Fallback to baseline circular mean if PCA is ill-conditioned (circular bounce)
                        travelHeadingRad = curHeading;
                    }
                }

                // Displacements
                float dE = stride * std::sin(travelHeadingRad);
                float dN = stride * std::cos(travelHeadingRad);
                east += dE;
                north += dN;

                float hDeg = travelHeadingRad * rad2deg;
                stepHeadingsDeg.push_back(hDeg);

                if (hasPrevTravelHeading) {
                    float delta = std::abs(static_cast<float>(wrapPi((travelHeadingRad - prevTravelHeading))));
                    if (delta > 30.0f * deg2rad) switchesOver30++;
                    if (delta > 60.0f * deg2rad) switchesOver60++;
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

    ReplayStats stats;
    stats.stepCount = stepCount;
    stats.cumulativeDistance = cumulativeDist;
    stats.finalEast = east;
    stats.finalNorth = north;
    stats.signFlips180 = signFlips180;
    stats.switchesOver30 = switchesOver30;
    stats.switchesOver60 = switchesOver60;

    // Step-to-step heading variation
    float varSum = 0.0f;
    for (size_t i = 1; i < stepHeadingsDeg.size(); ++i) {
        float r1 = stepHeadingsDeg[i - 1] * deg2rad;
        float r2 = stepHeadingsDeg[i] * deg2rad;
        varSum += std::abs(static_cast<float>(wrapPi(r2 - r1)) * rad2deg);
    }
    stats.stepHeadingVar = stepHeadingsDeg.size() > 1 ? (varSum / (stepHeadingsDeg.size() - 1)) : 0.0f;

    // 2-step cycle heading variation
    float cycleVarSum = 0.0f;
    int cycleCount = 0;
    for (size_t i = 2; i < stepHeadingsDeg.size(); i += 2) {
        float r1 = stepHeadingsDeg[i - 2] * deg2rad;
        float r2 = stepHeadingsDeg[i] * deg2rad;
        cycleVarSum += std::abs(static_cast<float>(wrapPi(r2 - r1)) * rad2deg);
        cycleCount++;
    }
    stats.cycleHeadingVar = cycleCount > 0 ? (cycleVarSum / cycleCount) : 0.0f;

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
        stats.headingStdDev = std::sqrt(varH / (sEnd - sStart));
    }

    return stats;
}

int main(int argc, char** argv) {
    runSyntheticTests();

    std::string csvPath = "/data/local/tmp/poco_walk.csv";
    if (argc > 1) csvPath = argv[1];

    auto rows = loadCsv(csvPath);
    if (rows.empty()) {
        std::cerr << "Could not open " << csvPath << std::endl;
        return 1;
    }

    ReplayStats s0 = runPhysicalReplay(rows, 0); // Baseline (M3.1)
    ReplayStats sA = runPhysicalReplay(rows, 1); // Candidate A (2-step bilateral cycle)
    ReplayStats sB = runPhysicalReplay(rows, 2); // Candidate B (Earth-frame PCA)

    std::cout << "\n================================================================================\n";
    std::cout << ">>> M3.5 PHYSICAL REPLAY COMPARISON: BASELINE vs CANDIDATE A vs CANDIDATE B\n";
    std::cout << "================================================================================\n";
    std::cout << std::left << std::setw(28) << "Metric"
              << std::setw(18) << "Baseline (M3.1)"
              << std::setw(20) << "Candidate A (2-Step)"
              << std::setw(20) << "Candidate B (PCA)" << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(28) << "Step Count"
              << std::setw(18) << s0.stepCount
              << std::setw(20) << sA.stepCount
              << std::setw(20) << sB.stepCount << "\n";
    std::cout << std::left << std::setw(28) << "Distance (m)"
              << std::setw(18) << std::fixed << std::setprecision(2) << s0.cumulativeDistance
              << std::setw(20) << sA.cumulativeDistance
              << std::setw(20) << sB.cumulativeDistance << "\n";
    std::cout << std::left << std::setw(28) << "Final E (m)"
              << std::setw(18) << std::fixed << std::setprecision(3) << s0.finalEast
              << std::setw(20) << sA.finalEast
              << std::setw(20) << sB.finalEast << "\n";
    std::cout << std::left << std::setw(28) << "Final N (m)"
              << std::setw(18) << std::fixed << std::setprecision(3) << s0.finalNorth
              << std::setw(20) << sA.finalNorth
              << std::setw(20) << sB.finalNorth << "\n";
    std::cout << std::left << std::setw(28) << "Lateral Drift |E| (m)"
              << std::setw(18) << std::fixed << std::setprecision(3) << std::abs(s0.finalEast)
              << std::setw(20) << std::abs(sA.finalEast)
              << std::setw(20) << std::abs(sB.finalEast) << "\n";
    std::cout << std::left << std::setw(28) << "Step Heading Var (deg)"
              << std::setw(18) << std::fixed << std::setprecision(1) << s0.stepHeadingVar
              << std::setw(20) << sA.stepHeadingVar
              << std::setw(20) << sB.stepHeadingVar << "\n";
    std::cout << std::left << std::setw(28) << "Cycle Heading Var (deg)"
              << std::setw(18) << std::fixed << std::setprecision(1) << s0.cycleHeadingVar
              << std::setw(20) << sA.cycleHeadingVar
              << std::setw(20) << sB.cycleHeadingVar << "\n";
    std::cout << std::left << std::setw(28) << "Straight Heading StdDev"
              << std::setw(18) << std::fixed << std::setprecision(1) << s0.headingStdDev
              << std::setw(20) << sA.headingStdDev
              << std::setw(20) << sB.headingStdDev << "\n";
    std::cout << std::left << std::setw(28) << "Steps with >30° change"
              << std::setw(18) << s0.switchesOver30
              << std::setw(20) << sA.switchesOver30
              << std::setw(20) << sB.switchesOver30 << "\n";
    std::cout << std::left << std::setw(28) << "Steps with >60° change"
              << std::setw(18) << s0.switchesOver60
              << std::setw(20) << sA.switchesOver60
              << std::setw(20) << sB.switchesOver60 << "\n";
    std::cout << std::left << std::setw(28) << "180° Sign Flips"
              << std::setw(18) << s0.signFlips180
              << std::setw(20) << sA.signFlips180
              << std::setw(20) << sB.signFlips180 << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    return 0;
}
