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

#include "ahrs/MathTypes.hpp"
#include "ahrs/Madgwick9DOF.hpp"
#include "ahrs/MagneticReliability.hpp"
#include "ahrs/StationaryDetector.hpp"
#include "pdr/StepDetector.hpp"
#include "pdr/StrideEstimator.hpp"
#include "ConstrainedCourseObserver.hpp"

// Sensor CSV Row representation
struct SensorRow {
    int64_t timestampNs;
    int sensorType;
    float x, y, z;
};

// CSV loader
static std::vector<SensorRow> loadCsv(const std::string& path) {
    std::vector<SensorRow> rows;
    std::ifstream file(path);
    if (!file.is_open()) return rows;
    std::string line;
    if (!std::getline(file, line)) return rows; // skip header

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

// Step-level telemetry record
struct StepRecord {
    uint32_t stepIndex;
    int64_t tNs;
    double tSec;
    float stride;
    float phoneHeadingDeg;
    float travelCourseDeg;
    float east;
    float north;
    bool isStationary;
    bool pcaAccepted;
    float pcaConfidence;
    float pcaLambda1;
    float pcaLambda2;
    float pcaOrientedAngleDeg;
};

// Comprehensive evaluation metrics for M3.7
struct M37ScenarioResult {
    std::string name;
    int mode; // 0 = M3.1 Baseline, 3 = M3.6 Observer
    uint32_t stepCount = 0;
    float totalDistance = 0.0f;
    float finalEast = 0.0f;
    float finalNorth = 0.0f;
    float netDisplacement = 0.0f;
    float courseVariation = 0.0f;         // Mean step-to-step absolute course change (deg)
    float straightCourseStdDev = 0.0f;    // StdDev of course during straight portions (deg)
    int jumpsOver30 = 0;
    int jumpsOver60 = 0;
    int signFlips180 = 0;
    float meanPcaConfidence = 0.0f;
    float pcaAcceptanceRate = 0.0f;
    float pcaRejectionRate = 0.0f;
    int acceptedPcaCount = 0;
    int rejectedPcaCount = 0;
    
    // Scenario-specific transition metrics
    int turnTrackingLatencySteps = -1;    // Steps to reach within 10° of target turn
    float maxOvershootDeg = 0.0f;         // Max overshoot past target
    float courseBeforeStopDeg = 0.0f;
    float courseDuringStopDeg = 0.0f;
    float phoneHeadingAfterRotationDeg = 0.0f;
    float courseImmediatelyAfterResumeDeg = 0.0f;
    float courseBeforeReversalDeg = 0.0f;
    float courseAfterReversalDeg = 0.0f;

    // M3.8A Dynamic Reversal Specific Metrics (Scenario D)
    uint32_t reversalTriggerStep = 0;
    float pcaConfidenceAtReversal = 0.0f;
    float phoneDivergenceAtReversalDeg = 0.0f;
    int consecutiveDivergenceStepsAtTrigger = 0;
    float courseImmediatelyBeforeReversalDeg = 0.0f;
    float courseImmediatelyAfterReversalDeg = 0.0f;
    int stepsToReachWithin20DegOfReverse = -1;

    std::vector<StepRecord> steps;
};

// Replay engine executing either M3.1 (mode=0) or M3.6 (mode=3)
static M37ScenarioResult runScenarioReplay(
    const std::vector<SensorRow>& rows,
    const std::string& scenarioName,
    int mode,
    float turnStartSec = -1.0f,
    float turnTargetCourseDeg = 0.0f,
    float stopStartSec = -1.0f,
    float stopEndSec = -1.0f)
{
    M37ScenarioResult result;
    result.name = scenarioName;
    result.mode = mode;

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
    float east = 0.0f, north = 0.0f, cumulativeDist = 0.0f;
    uint32_t stepCount = 0;

    int64_t lastAccelT = 0, lastGyroT = 0, lastMagT = 0;
    Vec3 latestAccel{0,0,0}, latestMag{0,0,0};
    bool init = false;

    std::vector<float> stepHeadingsDeg;
    float prevTravelHeadingRad = 0.0f;
    bool hasPrevTravelHeading = false;
    int jumpsOver30 = 0, jumpsOver60 = 0;

    double sumPcaConf = 0.0;
    int pcaCount = 0;
    int64_t t0 = rows.empty() ? 0 : rows[0].timestampNs;

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    constexpr float rad2deg = static_cast<float>(180.0 / M_PI);

    int turnStepTrigger = -1;

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
                ConstrainedCourseObserver::PcaOutput lastPca;

                int64_t startT = (lastStepTimeNs > 0 && (step.tNs - lastStepTimeNs) <= 2000000000LL)
                                 ? lastStepTimeNs : (step.tNs - 500000000LL);

                // Compute circular mean phone heading over step interval
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

                if (mode == 0) {
                    // M3.1 BASELINE: Circular mean phone heading
                    travelHeadingRad = stepPhoneHeading;
                } else if (mode == 3) {
                    // M3.6 CONSTRAINED COURSE OBSERVER
                    std::vector<std::pair<double, double>> winAcc;
                    for (const auto& a : earthAccelHistory) {
                        if (a.tNs >= startT && a.tNs <= step.tNs) winAcc.push_back({a.aE, a.aN});
                    }

                    travelHeadingRad = courseObs.update(winAcc, stepPhoneHeading, isStat);
                    lastPca = courseObs.getLastPca();
                    sumPcaConf += lastPca.confidence;
                    pcaCount++;
                }

                // Displacements
                float dE = stride * std::sin(travelHeadingRad);
                float dN = stride * std::cos(travelHeadingRad);
                east += dE;
                north += dN;

                float hDeg = travelHeadingRad * rad2deg;
                stepHeadingsDeg.push_back(hDeg);

                // Track jumps
                if (hasPrevTravelHeading) {
                    float diff = std::abs(static_cast<float>(wrapPi(travelHeadingRad - prevTravelHeadingRad)) * rad2deg);
                    if (diff > 30.0f) jumpsOver30++;
                    if (diff > 60.0f) jumpsOver60++;
                }
                prevTravelHeadingRad = travelHeadingRad;
                hasPrevTravelHeading = true;

                double stepSec = (step.tNs - t0) * 1e-9;
                StepRecord rec;
                rec.stepIndex = stepCount;
                rec.tNs = step.tNs;
                rec.tSec = stepSec;
                rec.stride = stride;
                rec.phoneHeadingDeg = stepPhoneHeading * rad2deg;
                rec.travelCourseDeg = hDeg;
                rec.east = east;
                rec.north = north;
                rec.isStationary = isStat;
                rec.pcaAccepted = lastPca.accepted;
                rec.pcaConfidence = lastPca.confidence;
                rec.pcaLambda1 = lastPca.lambda1;
                rec.pcaLambda2 = lastPca.lambda2;
                rec.pcaOrientedAngleDeg = lastPca.orientedAngleRad * rad2deg;
                result.steps.push_back(rec);

                // Check turn latency
                if (turnStartSec > 0.0f && stepSec >= turnStartSec) {
                    if (turnStepTrigger < 0) turnStepTrigger = stepCount;
                    float err = std::abs(static_cast<float>(wrapPi((hDeg - turnTargetCourseDeg) * deg2rad)) * rad2deg);
                    if (err <= 10.0f && result.turnTrackingLatencySteps < 0) {
                        result.turnTrackingLatencySteps = (stepCount - turnStepTrigger);
                    }
                    if (err > result.maxOvershootDeg && stepSec > turnStartSec + 3.0f) {
                        result.maxOvershootDeg = err;
                    }
                }

                // Stop & resume metrics
                if (stopStartSec > 0.0f && stepSec < stopStartSec) {
                    result.courseBeforeStopDeg = hDeg;
                }
                if (stopEndSec > 0.0f && stepSec >= stopEndSec && result.courseImmediatelyAfterResumeDeg == 0.0f) {
                    result.courseImmediatelyAfterResumeDeg = hDeg;
                }

                // Reversal metrics (Scenario D)
                if (mode == 3) {
                    if (courseObs.isReversalActive()) {
                        if (result.reversalTriggerStep == 0) {
                            result.reversalTriggerStep = stepCount;
                            result.pcaConfidenceAtReversal = lastPca.confidence;
                            result.phoneDivergenceAtReversalDeg = courseObs.getLastPhoneDivergenceDeg();
                            result.consecutiveDivergenceStepsAtTrigger = courseObs.getConsecutiveDivergentSteps();
                            result.courseImmediatelyBeforeReversalDeg = stepHeadingsDeg.size() > 1 ? stepHeadingsDeg[stepHeadingsDeg.size() - 2] : hDeg;
                        }
                    }
                    if (result.reversalTriggerStep > 0 && result.stepsToReachWithin20DegOfReverse < 0) {
                        float targetRevDeg = static_cast<float>(wrapPi((result.courseImmediatelyBeforeReversalDeg + 180.0f) * deg2rad) * rad2deg);
                        float errRev = std::abs(static_cast<float>(wrapPi((hDeg - targetRevDeg) * deg2rad)) * rad2deg);
                        if (errRev <= 20.0f) {
                            result.stepsToReachWithin20DegOfReverse = static_cast<int>(stepCount - result.reversalTriggerStep);
                            result.courseImmediatelyAfterReversalDeg = hDeg;
                        }
                    }
                }

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

            double curSec = (r.timestampNs - t0) * 1e-9;
            if (stopStartSec > 0.0f && stopEndSec > 0.0f && curSec >= stopStartSec && curSec <= stopEndSec) {
                result.courseDuringStopDeg = (mode == 3) ? courseObs.getTravelCourseDeg() : (phoneForwardBearing(ahrs.q) * rad2deg);
                result.phoneHeadingAfterRotationDeg = static_cast<float>(phoneForwardBearing(ahrs.q) * rad2deg);
            }
        } else if (r.sensorType == 2) { // MAG
            latestMag = {r.x, r.y, r.z};
            lastMagT = r.timestampNs;
            magRel.update(r.x, r.y, r.z, ahrs.q, statDet.isStationary());
        }
    }

    result.stepCount = stepCount;
    result.totalDistance = cumulativeDist;
    result.finalEast = east;
    result.finalNorth = north;
    result.netDisplacement = std::hypot(east, north);
    result.jumpsOver30 = jumpsOver30;
    result.jumpsOver60 = jumpsOver60;
    result.signFlips180 = (mode == 3) ? courseObs.getSignFlips180() : 0;
    result.acceptedPcaCount = (mode == 3) ? courseObs.getAcceptedPcaUpdates() : 0;
    result.rejectedPcaCount = (mode == 3) ? courseObs.getRejectedPcaUpdates() : 0;
    int totalPca = result.acceptedPcaCount + result.rejectedPcaCount;
    result.pcaAcceptanceRate = (totalPca > 0) ? (result.acceptedPcaCount * 100.0f / totalPca) : 0.0f;
    result.pcaRejectionRate = (totalPca > 0) ? (result.rejectedPcaCount * 100.0f / totalPca) : 0.0f;
    result.meanPcaConfidence = (pcaCount > 0) ? static_cast<float>(sumPcaConf / pcaCount) : 0.0f;

    // Course step-to-step variation
    float varSum = 0.0f;
    for (size_t i = 1; i < stepHeadingsDeg.size(); ++i) {
        float r1 = stepHeadingsDeg[i - 1] * deg2rad;
        float r2 = stepHeadingsDeg[i] * deg2rad;
        varSum += std::abs(static_cast<float>(wrapPi(r2 - r1)) * rad2deg);
    }
    result.courseVariation = stepHeadingsDeg.size() > 1 ? (varSum / (stepHeadingsDeg.size() - 1)) : 0.0f;

    // Straight-course standard deviation (first 12 steps)
    if (stepHeadingsDeg.size() >= 5) {
        size_t sEnd = std::min(size_t(12), stepHeadingsDeg.size());
        float meanH = 0.0f;
        for (size_t i = 0; i < sEnd; ++i) meanH += stepHeadingsDeg[i];
        meanH /= sEnd;
        float varH = 0.0f;
        for (size_t i = 0; i < sEnd; ++i) {
            float d = stepHeadingsDeg[i] - meanH;
            varH += d * d;
        }
        result.straightCourseStdDev = std::sqrt(varH / sEnd);
    }

    return result;
}

// Helper to format side-by-side comparison table
static void printScenarioComparison(const M37ScenarioResult& m31, const M37ScenarioResult& m36) {
    std::cout << "\n--------------------------------------------------------------------------------\n";
    std::cout << ">>> COMPARISON TABLE: " << m31.name << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(32) << "Metric"
              << std::setw(24) << "M3.1 (Baseline)"
              << std::setw(24) << "M3.6 (Observer)"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    auto printRow = [](const std::string& name, const std::string& v1, const std::string& v2) {
        std::cout << std::left << std::setw(32) << name
                  << std::setw(24) << v1
                  << std::setw(24) << v2 << "\n";
    };

    auto fmtFloat = [](float val, int prec = 2, const std::string& unit = "") {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec) << val << unit;
        return ss.str();
    };

    printRow("1. Step Count", std::to_string(m31.stepCount), std::to_string(m36.stepCount));
    printRow("2. Total Distance", fmtFloat(m31.totalDistance, 2, " m"), fmtFloat(m36.totalDistance, 2, " m"));
    printRow("3. Final East", fmtFloat(m31.finalEast, 3, " m"), fmtFloat(m36.finalEast, 3, " m"));
    printRow("4. Final North", fmtFloat(m31.finalNorth, 3, " m"), fmtFloat(m36.finalNorth, 3, " m"));
    printRow("5. Net Displacement", fmtFloat(m31.netDisplacement, 2, " m"), fmtFloat(m36.netDisplacement, 2, " m"));
    printRow("6. Course Variation", fmtFloat(m31.courseVariation, 1, "°"), fmtFloat(m36.courseVariation, 1, "°"));
    printRow("7. Straight Course StdDev", fmtFloat(m31.straightCourseStdDev, 1, "°"), fmtFloat(m36.straightCourseStdDev, 1, "°"));
    printRow("8. Jumps > 30°", std::to_string(m31.jumpsOver30), std::to_string(m36.jumpsOver30));
    printRow("9. Jumps > 60°", std::to_string(m31.jumpsOver60), std::to_string(m36.jumpsOver60));
    printRow("10. 180° Flips", std::to_string(m31.signFlips180), std::to_string(m36.signFlips180));
    printRow("11. Mean PCA Confidence", "N/A", fmtFloat(m36.meanPcaConfidence, 3));
    printRow("12. PCA Acceptance Rate", "N/A", fmtFloat(m36.pcaAcceptanceRate, 1, "%") + " (" + std::to_string(m36.acceptedPcaCount) + ")");
    printRow("13. PCA Rejection Rate", "N/A", fmtFloat(m36.pcaRejectionRate, 1, "%") + " (" + std::to_string(m36.rejectedPcaCount) + ")");
    
    if (m36.turnTrackingLatencySteps >= 0) {
        printRow("14. Turn Tracking Latency", "Instant (Coupled)", std::to_string(m36.turnTrackingLatencySteps) + " steps");
        printRow("15. Max Overshoot", "N/A", fmtFloat(m36.maxOvershootDeg, 1, "°"));
    }
    if (m36.courseBeforeStopDeg != 0.0f || m36.courseImmediatelyAfterResumeDeg != 0.0f) {
        printRow("16a. Course Before Stop", fmtFloat(m31.courseBeforeStopDeg, 1, "°"), fmtFloat(m36.courseBeforeStopDeg, 1, "°"));
        printRow("16b. Course During Stop", fmtFloat(m31.courseDuringStopDeg, 1, "°"), fmtFloat(m36.courseDuringStopDeg, 1, "°"));
        printRow("16c. Phone Heading Post-Rotate", fmtFloat(m31.phoneHeadingAfterRotationDeg, 1, "°"), fmtFloat(m36.phoneHeadingAfterRotationDeg, 1, "°"));
        printRow("16d. Course After Resume", fmtFloat(m31.courseImmediatelyAfterResumeDeg, 1, "°"), fmtFloat(m36.courseImmediatelyAfterResumeDeg, 1, "°"));
    }
    if (m36.reversalTriggerStep > 0) {
        printRow("17a. Reversal Trigger Step", "N/A", "Step " + std::to_string(m36.reversalTriggerStep));
        printRow("17b. Divergence Persistence", "N/A", std::to_string(m36.consecutiveDivergenceStepsAtTrigger) + " steps");
        printRow("17c. Phone Divergence at Trigger", "N/A", fmtFloat(m36.phoneDivergenceAtReversalDeg, 1, "°"));
        printRow("17d. PCA Conf at Reversal", "N/A", fmtFloat(m36.pcaConfidenceAtReversal, 3));
        printRow("17e. Course Before Reversal", fmtFloat(m31.courseBeforeReversalDeg, 1, "°"), fmtFloat(m36.courseImmediatelyBeforeReversalDeg, 1, "°"));
        printRow("17f. Course After Reversal", fmtFloat(m31.courseAfterReversalDeg, 1, "°"), fmtFloat(m36.courseImmediatelyAfterReversalDeg, 1, "°"));
        printRow("17g. Steps to within 20° Reverse", "N/A", std::to_string(m36.stepsToReachWithin20DegOfReverse) + " steps");
    }
    std::cout << "--------------------------------------------------------------------------------\n";
}

int main(int argc, char** argv) {
    std::cout << "================================================================================\n";
    std::cout << "  MILESTONE 3.7: CONTROLLED PHYSICAL VALIDATION OF M3.6 COURSE OBSERVER\n";
    std::cout << "  DIAGNOSTIC-ONLY EXECUTION — POCO X6 PRO SENSOR VALIDATION HARNESS\n";
    std::cout << "================================================================================\n";

    std::string basePath = "/data/local/tmp/";
    if (argc > 1) {
        basePath = argv[1];
        if (basePath.back() != '/' && basePath.back() != '\\') basePath += "/";
    }

    struct ScenarioDef {
        std::string id;
        std::string filename;
        std::string desc;
        float turnStartSec;
        float turnTargetCourseDeg;
        float stopStartSec;
        float stopEndSec;
    };

    std::vector<ScenarioDef> scenarioList = {
        {"SCENARIO A", "m37_A_straight.csv", "Straight walk / normal handheld texting posture", -1.0f, 0.0f, -1.0f, -1.0f},
        {"SCENARIO B", "m37_B_phone_yaw.csv", "Straight walk + deliberate phone yaw swing +-55 deg", -1.0f, 0.0f, -1.0f, -1.0f},
        {"SCENARIO C", "m37_C_turn90.csv", "Continuous walking 90 deg turn to East over ~4 steps", 10.0f, 90.0f, -1.0f, -1.0f},
        {"SCENARIO D", "m37_D_reverse180.csv", "Genuine 180 deg walking reversal over ~5 steps", 11.0f, 180.0f, -1.0f, -1.0f},
        {"SCENARIO E", "m37_E_stop_rotate_resume.csv", "Walk -> Stop 3.5s (rotate phone 90 deg) -> Resume walk", -1.0f, 0.0f, 8.5f, 12.0f},
        {"SCENARIO F1", "m37_F_phone_offset45.csv", "Straight walk with phone held fixed at +45 deg yaw", -1.0f, 0.0f, -1.0f, -1.0f},
        {"SCENARIO F2", "m37_F_phone_offset90.csv", "Straight walk with phone held fixed at +90 deg yaw", -1.0f, 0.0f, -1.0f, -1.0f},
        {"SCENARIO G", "m37_G_varied_posture.csv", "Dangling carry at side with arm swing dynamics", -1.0f, 0.0f, -1.0f, -1.0f},
    };

    std::vector<std::pair<M37ScenarioResult, M37ScenarioResult>> allResults;

    for (const auto& sc : scenarioList) {
        std::string fullPath = basePath + sc.filename;
        auto rows = loadCsv(fullPath);
        if (rows.empty()) {
            std::cerr << "WARNING: Could not load " << fullPath << " (skipping)\n";
            continue;
        }

        std::cout << "\n>>> Executing " << sc.id << " (" << sc.desc << ")...\n";
        std::cout << "    Loaded " << rows.size() << " samples from " << fullPath << "\n";

        M37ScenarioResult resM31 = runScenarioReplay(rows, sc.id + " (" + sc.filename + ")", 0, sc.turnStartSec, sc.turnTargetCourseDeg, sc.stopStartSec, sc.stopEndSec);
        M37ScenarioResult resM36 = runScenarioReplay(rows, sc.id + " (" + sc.filename + ")", 3, sc.turnStartSec, sc.turnTargetCourseDeg, sc.stopStartSec, sc.stopEndSec);

        printScenarioComparison(resM31, resM36);
        if (sc.id == "SCENARIO D") {
            std::cout << "\n>>> SCENARIO D STEP-BY-STEP REVERSAL TRANSITION (M3.6):\n";
            for (const auto& st : resM36.steps) {
                if (st.stepIndex >= 18 && st.stepIndex <= 30) {
                    std::cout << "  Step " << std::setw(2) << st.stepIndex 
                              << " (t=" << std::fixed << std::setprecision(2) << st.tSec << "s): "
                              << " Phone=" << std::setw(6) << std::setprecision(1) << st.phoneHeadingDeg << "°,"
                              << " Course=" << std::setw(6) << std::setprecision(1) << st.travelCourseDeg << "°,"
                              << " PCA=" << std::setw(6) << std::setprecision(1) << st.pcaOrientedAngleDeg << "°,"
                              << " Conf=" << std::setprecision(2) << st.pcaConfidence
                              << " E=" << std::setprecision(2) << st.east << "m, N=" << st.north << "m\n";
                }
            }
        }
        allResults.push_back({resM31, resM36});
    }

    // Benchmark Reference: POCO physical walk (poco_walk.csv)
    std::string pocoWalkPath = basePath + "poco_walk.csv";
    auto pocoRows = loadCsv(pocoWalkPath);
    if (!pocoRows.empty()) {
        std::cout << "\n>>> Executing REFERENCE BENCHMARK: POCO Physical Walk Recording (poco_walk.csv)...\n";
        M37ScenarioResult pM31 = runScenarioReplay(pocoRows, "REFERENCE: poco_walk.csv (87 steps)", 0);
        M37ScenarioResult pM36 = runScenarioReplay(pocoRows, "REFERENCE: poco_walk.csv (87 steps)", 3);
        printScenarioComparison(pM31, pM36);
    }

    std::cout << "\n================================================================================\n";
    std::cout << ">>> M3.7 CONTROLLED VALIDATION EXECUTION COMPLETE!\n";
    std::cout << "================================================================================\n";

    return 0;
}
