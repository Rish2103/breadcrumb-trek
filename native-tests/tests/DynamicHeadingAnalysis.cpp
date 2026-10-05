#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include "ahrs/NavigationEngine.hpp"
#include "ahrs/MathTypes.hpp"
#include "ahrs/MagneticReliability.hpp"
#include "pdr/PdrEngine.hpp"

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

struct UpdateLog {
    int64_t tNs;
    double tRelSec;
    // Accel
    Vec3 accel;
    double aMag;
    double aDiffG;
    // Gyro
    Vec3 gyro;
    double gMag;
    double earthYawRateDeg; // Clockwise heading rate in deg/s
    // Mag
    Vec3 mag;
    double mMag;
    float Cmag;
    bool isMagCalibrated;
    float b0;
    // AHRS
    Quaternion q;
    double headingDeg;
    double deltaHeadingAhrsDeg;
    double deltaHeadingGyroDeg;
    // PDR
    bool stepDetected;
    uint32_t stepIndex;
    float stride;
    float dE, dN;
    float cumE, cumN;
};

int main(int argc, char** argv) {
    std::string csvPath = "/data/local/tmp/poco_walk.csv";
    if (argc > 1) {
        csvPath = argv[1];
    }

    auto rows = loadCsv(csvPath);
    if (rows.empty()) {
        std::cerr << "Could not open " << csvPath << std::endl;
        return 1;
    }

    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();
    engine.startTrailRecording();

    std::vector<UpdateLog> logs;
    logs.reserve(rows.size());

    int64_t t0 = rows[0].timestampNs;
    double prevHeadingRad = 0.0;
    bool hasPrevHeading = false;
    uint32_t prevSteps = 0;
    float prevCumE = 0.0f, prevCumN = 0.0f;

    Vec3 latestA{0,0,0};
    Vec3 latestM{0,0,0};
    int64_t lastGyroT = 0;

    for (const auto& r : rows) {
        if (r.sensorType == 1) { // ACCEL
            latestA = {r.x, r.y, r.z};
            engine.pushAccel(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 2) { // MAG
            latestM = {r.x, r.y, r.z};
            engine.pushMag(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 4) { // GYRO
            double dt = (lastGyroT > 0) ? (r.timestampNs - lastGyroT) * 1e-9 : 0.0;
            lastGyroT = r.timestampNs;

            engine.pushGyro(r.x, r.y, r.z, r.timestampNs);

            Quaternion q = engine.getQuaternion();
            double headingRad = phoneForwardBearing(q);
            double headingDeg = headingRad * 180.0 / M_PI;

            Vec3 bias = engine.getGyroBias();
            Vec3 gCorrected{r.x - bias.x, r.y - bias.y, r.z - bias.z};
            Vec3 omegaEarth = rotateBodyToEarth(q, gCorrected);
            // In NWU: Up is +z. Clockwise rotation rate from North towards East (-West) is -omegaEarth.z
            double earthYawRateDeg = -omegaEarth.z * 180.0 / M_PI;

            double deltaAhrsDeg = 0.0;
            double deltaGyroDeg = 0.0;
            if (hasPrevHeading && dt > 0.0 && dt < 0.25) {
                deltaAhrsDeg = wrapPi(headingRad - prevHeadingRad) * 180.0 / M_PI;
                deltaGyroDeg = earthYawRateDeg * dt;
            }

            float state[40];
            engine.getState(state, 40);
            uint32_t curSteps = static_cast<uint32_t>(state[8]);
            bool stepDetected = (curSteps > prevSteps);

            float stride = 0.0f, dE = 0.0f, dN = 0.0f;
            float curCumE = state[0];
            float curCumN = state[1];
            if (stepDetected) {
                stride = engine.getPdrEngine().getLastStride();
                dE = curCumE - prevCumE;
                dN = curCumN - prevCumN;
                prevCumE = curCumE;
                prevCumN = curCumN;
                prevSteps = curSteps;
            }

            double aMag = std::sqrt(latestA.x*latestA.x + latestA.y*latestA.y + latestA.z*latestA.z);
            double mMag = std::sqrt(latestM.x*latestM.x + latestM.y*latestM.y + latestM.z*latestM.z);

            logs.push_back({
                r.timestampNs,
                (r.timestampNs - t0) * 1e-9,
                latestA,
                aMag,
                std::abs(aMag - 9.80665),
                {r.x, r.y, r.z},
                std::sqrt(r.x*r.x + r.y*r.y + r.z*r.z),
                earthYawRateDeg,
                latestM,
                mMag,
                engine.getCmag(),
                engine.isMagCalibrated(),
                engine.getCalibratedB0(),
                q,
                headingDeg,
                deltaAhrsDeg,
                deltaGyroDeg,
                stepDetected,
                curSteps,
                stride,
                dE, dN,
                curCumE, curCumN
            });

            prevHeadingRad = headingRad;
            hasPrevHeading = true;
        }
    }

    std::cout << "Processed " << logs.size() << " AHRS updates from " << csvPath << "\n";

    // =========================================================================
    // PHASE 2 & 3: DETAILED ANALYSIS OF SEGMENT 1 (t = 1.4s to 2.6s)
    // =========================================================================
    std::cout << "\n================================================================================\n";
    std::cout << ">>> PHASE 2: STEP-LEVEL TELEMETRY TABLE (t = 1.4s to 3.2s)\n";
    std::cout << "================================================================================\n";
    std::cout << std::left 
              << std::setw(7) << "t(s)"
              << std::setw(10) << "Heading"
              << std::setw(8) << "gz"
              << std::setw(9) << "|gyro|"
              << std::setw(9) << "|accel|"
              << std::setw(9) << "|mag|"
              << std::setw(7) << "Cmag"
              << std::setw(6) << "Step"
              << std::setw(8) << "dE(m)"
              << std::setw(8) << "dN(m)"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    for (const auto& l : logs) {
        if (l.tRelSec >= 1.4 && l.tRelSec <= 3.2) {
            if (l.stepDetected || std::abs(l.deltaHeadingAhrsDeg) > 5.0 || ((int(l.tRelSec*100) % 20) == 0)) {
                std::cout << std::fixed << std::setprecision(2)
                          << std::setw(7) << l.tRelSec
                          << std::setw(10) << std::setprecision(1) << l.headingDeg
                          << std::setw(8) << std::setprecision(2) << l.gyro.z
                          << std::setw(9) << l.gMag
                          << std::setw(9) << l.aMag
                          << std::setw(9) << l.mMag
                          << std::setw(7) << std::setprecision(2) << l.Cmag
                          << std::setw(6) << (l.stepDetected ? std::to_string(l.stepIndex) : "-")
                          << std::setw(8) << std::setprecision(2) << l.dE
                          << std::setw(8) << std::setprecision(2) << l.dN
                          << "\n";
            }
        }
    }

    // =========================================================================
    // PHASE 3: SEPARATE REAL ROTATION FROM FUSION ERROR
    // Compare cumulative AHRS heading change vs cumulative integrated Gyro yaw
    // =========================================================================
    std::cout << "\n================================================================================\n";
    std::cout << ">>> PHASE 3: INTEGRATED GYRO YAW vs AHRS HEADING CHANGE\n";
    std::cout << "================================================================================\n";

    // Segment 1: t=1.4s to 2.6s
    {
        double sumGyroYawDeg = 0.0;
        double sumAhrsDeltaDeg = 0.0;
        double hStart = 0.0, hEnd = 0.0;
        bool first = true;
        for (const auto& l : logs) {
            if (l.tRelSec >= 1.4 && l.tRelSec <= 2.6) {
                if (first) { hStart = l.headingDeg; first = false; }
                hEnd = l.headingDeg;
                sumGyroYawDeg += l.deltaHeadingGyroDeg;
                sumAhrsDeltaDeg += l.deltaHeadingAhrsDeg;
            }
        }
        constexpr double deg2rad = M_PI / 180.0;
        constexpr double rad2deg = 180.0 / M_PI;
        double netAhrsChange = wrapPi((hEnd - hStart) * deg2rad) * rad2deg;

        std::cout << "Segment t = [1.4s, 2.6s] (Steps 3-5):\n";
        std::cout << "  Start Heading:         " << std::fixed << std::setprecision(1) << hStart << " deg\n";
        std::cout << "  End Heading:           " << hEnd << " deg\n";
        std::cout << "  Net AHRS Heading Delta: " << netAhrsChange << " deg\n";
        std::cout << "  Integrated Gyro Yaw:   " << sumGyroYawDeg << " deg\n";
        std::cout << "  Discrepancy (Fusion):  " << (netAhrsChange - sumGyroYawDeg) << " deg\n";
    }

    // Segment 2: t=3.0s to 5.0s (Steps 6-9)
    {
        double sumGyroYawDeg = 0.0;
        double hStart = 0.0, hEnd = 0.0;
        bool first = true;
        for (const auto& l : logs) {
            if (l.tRelSec >= 3.0 && l.tRelSec <= 5.0) {
                if (first) { hStart = l.headingDeg; first = false; }
                hEnd = l.headingDeg;
                sumGyroYawDeg += l.deltaHeadingGyroDeg;
            }
        }
        constexpr double deg2rad = M_PI / 180.0;
        constexpr double rad2deg = 180.0 / M_PI;
        double netAhrsChange = wrapPi((hEnd - hStart) * deg2rad) * rad2deg;

        std::cout << "\nSegment t = [3.0s, 5.0s] (Steps 6-9):\n";
        std::cout << "  Start Heading:         " << std::fixed << std::setprecision(1) << hStart << " deg\n";
        std::cout << "  End Heading:           " << hEnd << " deg\n";
        std::cout << "  Net AHRS Heading Delta: " << netAhrsChange << " deg\n";
        std::cout << "  Integrated Gyro Yaw:   " << sumGyroYawDeg << " deg\n";
        std::cout << "  Discrepancy (Fusion):  " << (netAhrsChange - sumGyroYawDeg) << " deg\n";
    }

    // Segment 3: Complete Walk Overall (t = 0 to 154s)
    {
        double totalAbsAhrsDelta = 0.0;
        double totalAbsGyroDelta = 0.0;
        double totalDiscrepancy = 0.0;
        size_t count = 0;

        for (size_t i = 1; i < logs.size(); ++i) {
            totalAbsAhrsDelta += std::abs(logs[i].deltaHeadingAhrsDeg);
            totalAbsGyroDelta += std::abs(logs[i].deltaHeadingGyroDeg);
            totalDiscrepancy += std::abs(logs[i].deltaHeadingAhrsDeg - logs[i].deltaHeadingGyroDeg);
            count++;
        }
        std::cout << "\nComplete Walk Telemetry Summary (8,129 samples):\n";
        std::cout << "  Mean |delta Heading AHRS| per sample: " << (totalAbsAhrsDelta / count) << " deg\n";
        std::cout << "  Mean |delta Heading Gyro| per sample: " << (totalAbsGyroDelta / count) << " deg\n";
        std::cout << "  Mean Discrepancy (Fusion Error) per sample: " << (totalDiscrepancy / count) << " deg\n";
        std::cout << "  Correlation Ratio (AHRS motion / Gyro motion): " << (totalAbsAhrsDelta / totalAbsGyroDelta) * 100.0 << "%\n";
    }

    // =========================================================================
    // PHASE 4: MAGNETOMETER & CMAG ANALYSIS
    // =========================================================================
    std::cout << "\n================================================================================\n";
    std::cout << ">>> PHASE 4 & 6: MAGNETOMETER RELIABILITY & MADGWICK MODE\n";
    std::cout << "================================================================================\n";
    {
        size_t cMagZeroCount = 0;
        size_t cMagHighCount = 0;
        double meanCmag = 0.0;
        double minMag = 1e9, maxMag = -1e9;
        double sumMag = 0.0;

        for (const auto& l : logs) {
            meanCmag += l.Cmag;
            if (l.Cmag < 0.001f) cMagZeroCount++;
            if (l.Cmag > 0.5f) cMagHighCount++;

            if (l.mMag < minMag) minMag = l.mMag;
            if (l.mMag > maxMag) maxMag = l.mMag;
            sumMag += l.mMag;
        }
        meanCmag /= logs.size();
        double meanMag = sumMag / logs.size();

        std::cout << "Magnetometer Calibration Established: " << (logs.back().isMagCalibrated ? "YES" : "NO") << "\n";
        std::cout << "Calibrated B0:                        " << logs.back().b0 << " uT\n";
        std::cout << "Mean Magnetic Field Magnitude:        " << meanMag << " uT (Min: " << minMag << ", Max: " << maxMag << ")\n";
        std::cout << "Mean Cmag:                            " << meanCmag << "\n";
        std::cout << "Samples with Cmag == 0.0 (IMU-only):   " << cMagZeroCount << " (" << (cMagZeroCount * 100.0 / logs.size()) << "%)\n";
        std::cout << "Samples with Cmag > 0.5 (Mag-active):  " << cMagHighCount << " (" << (cMagHighCount * 100.0 / logs.size()) << "%)\n";
        std::cout << "Madgwick Gain beta:                   0.08\n";
    }

    // =========================================================================
    // PHASE 5: ACCELEROMETER / DYNAMIC MOTION ANALYSIS
    // =========================================================================
    std::cout << "\n================================================================================\n";
    std::cout << ">>> PHASE 5: DYNAMIC ACCELERATION & GAIT CORRELATION\n";
    std::cout << "================================================================================\n";
    {
        double maxA = 0.0, maxDiffG = 0.0;
        double meanDiffG = 0.0;
        for (const auto& l : logs) {
            if (l.aMag > maxA) maxA = l.aMag;
            if (l.aDiffG > maxDiffG) maxDiffG = l.aDiffG;
            meanDiffG += l.aDiffG;
        }
        meanDiffG /= logs.size();
        std::cout << "Max |a|:               " << maxA << " m/s²\n";
        std::cout << "Max ||a| - g|:         " << maxDiffG << " m/s²\n";
        std::cout << "Mean Dynamic Bounce:   " << meanDiffG << " m/s²\n";
    }

    // =========================================================================
    // PHASE 7: CONTROL REPLAYS
    // =========================================================================
    std::cout << "\n================================================================================\n";
    std::cout << ">>> PHASE 7: CONTROL REPLAYS (STATIC vs WALKING)\n";
    std::cout << "================================================================================\n";
    // Control A: Stationary segment (first 1.0s before walking begins)
    {
        double sumH = 0.0, sumG = 0.0, sumA = 0.0, sumM = 0.0, sumC = 0.0;
        size_t count = 0;
        std::vector<double> hVec;
        for (const auto& l : logs) {
            if (l.tRelSec <= 1.0) {
                sumH += l.headingDeg;
                sumG += l.gMag;
                sumA += l.aMag;
                sumM += l.mMag;
                sumC += l.Cmag;
                hVec.push_back(l.headingDeg);
                count++;
            }
        }
        double meanH = count > 0 ? (sumH / count) : 0.0;
        double varH = 0.0;
        for (double h : hVec) varH += (h - meanH) * (h - meanH);
        double stdH = count > 1 ? std::sqrt(varH / count) : 0.0;

        std::cout << "CONTROL A (STATIC PHONE - t <= 1.0s):\n";
        std::cout << "  Heading StdDev:   " << stdH << " deg\n";
        std::cout << "  Mean |gyro|:      " << (sumG / count) << " rad/s\n";
        std::cout << "  Mean |accel|:     " << (sumA / count) << " m/s²\n";
        std::cout << "  Mean |mag|:       " << (sumM / count) << " uT\n";
        std::cout << "  Mean Cmag:        " << (sumC / count) << "\n";
    }

    // Control C: Walking segment (t = 2.0s to 12.0s)
    {
        double sumH = 0.0, sumG = 0.0, sumA = 0.0, sumM = 0.0, sumC = 0.0;
        size_t count = 0;
        std::vector<double> hVec;
        for (const auto& l : logs) {
            if (l.tRelSec >= 2.0 && l.tRelSec <= 12.0) {
                sumH += l.headingDeg;
                sumG += l.gMag;
                sumA += l.aMag;
                sumM += l.mMag;
                sumC += l.Cmag;
                hVec.push_back(l.headingDeg);
                count++;
            }
        }
        double meanH = count > 0 ? (sumH / count) : 0.0;
        double varH = 0.0;
        for (double h : hVec) varH += (h - meanH) * (h - meanH);
        double stdH = count > 1 ? std::sqrt(varH / count) : 0.0;

        std::cout << "\nCONTROL C (ACTIVE WALKING - t = [2.0s, 12.0s]):\n";
        std::cout << "  Heading StdDev:   " << stdH << " deg\n";
        std::cout << "  Mean |gyro|:      " << (sumG / count) << " rad/s\n";
        std::cout << "  Mean |accel|:     " << (sumA / count) << " m/s²\n";
        std::cout << "  Mean |mag|:       " << (sumM / count) << " uT\n";
        std::cout << "  Mean Cmag:        " << (sumC / count) << "\n";
    }

    return 0;
}
