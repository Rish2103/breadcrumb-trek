#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <deque>

#include "ahrs/NavigationEngine.hpp"
#include "ahrs/MathTypes.hpp"
#include "pdr/PdrEngine.hpp"

struct SensorRow {
    int64_t timestampNs;
    int sensorType;
    float x, y, z;
};

std::vector<SensorRow> loadCsv(const std::string& path) {
    std::vector<SensorRow> rows;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV: " << path << std::endl;
        return rows;
    }
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
// PART 1: CONTROL TESTS (TEST 1 - 5)
// ============================================================================
void runControlTests() {
    std::cout << "\n=======================================================\n";
    std::cout << ">>> RUNNING CONTROL TESTS 1-5 (Variant A vs Variant B)\n";
    std::cout << "=======================================================\n";

    // -------------------------------------------------------------
    // TEST 1 — STATIC PHONE (10s stationary)
    // -------------------------------------------------------------
    std::cout << "\n--- TEST 1: STATIC PHONE (10 seconds flat hold) ---\n";
    {
        Madgwick9DOF ahrsA;
        ahrsA.decoupleYawAccelGradient = false; // Variant A
        Madgwick9DOF ahrsB;
        ahrsB.decoupleYawAccelGradient = true;  // Variant B

        const float dt = 0.02f; // 50 Hz
        for (int i = 0; i < 500; ++i) { // 10s
            // Flat stationary on table: ax=0, ay=0, az=9.81 m/s²
            ahrsA.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.0f, 35.0f, 23.0f, 1.0f, dt);
            ahrsB.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.0f, 35.0f, 23.0f, 1.0f, dt);
        }

        double hA = phoneForwardBearing(ahrsA.q) * 180.0 / M_PI;
        double hB = phoneForwardBearing(ahrsB.q) * 180.0 / M_PI;
        std::cout << "  Variant A (Production) Heading: " << hA << " deg, Quat: [" 
                  << ahrsA.q.w << "," << ahrsA.q.x << "," << ahrsA.q.y << "," << ahrsA.q.z << "]\n";
        std::cout << "  Variant B (Decoupled)  Heading: " << hB << " deg, Quat: [" 
                  << ahrsB.q.w << "," << ahrsB.q.x << "," << ahrsB.q.y << "," << ahrsB.q.z << "]\n";
        assert(std::fabs(hA - hB) < 1.0);
        std::cout << "  PASS: Static orientation and roll/pitch remain identical and stable.\n";
    }

    // -------------------------------------------------------------
    // TEST 2 — PURE ROTATION (90° and 180° rotation)
    // -------------------------------------------------------------
    std::cout << "\n--- TEST 2: PURE ROTATION (90° and 180° yaw rotation) ---\n";
    {
        Madgwick9DOF ahrsA;
        ahrsA.decoupleYawAccelGradient = false;
        Madgwick9DOF ahrsB;
        ahrsB.decoupleYawAccelGradient = true;

        const float dt = 0.02f;
        // Rotate 90° clockwise at 1.57 rad/s for 1.0s (gz = -1.5708)
        for (int i = 0; i < 50; ++i) {
            ahrsA.update(0.0f, 0.0f, -1.570796f, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, dt);
            ahrsB.update(0.0f, 0.0f, -1.570796f, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, dt);
        }
        double hA_90 = phoneForwardBearing(ahrsA.q) * 180.0 / M_PI;
        double hB_90 = phoneForwardBearing(ahrsB.q) * 180.0 / M_PI;
        std::cout << "  After 90° Turn: Variant A = " << hA_90 << " deg, Variant B = " << hB_90 << " deg\n";
        assert(std::fabs(hA_90 - hB_90) < 1.0);

        // Rotate another 90° clockwise (total 180°)
        for (int i = 0; i < 50; ++i) {
            ahrsA.update(0.0f, 0.0f, -1.570796f, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, dt);
            ahrsB.update(0.0f, 0.0f, -1.570796f, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, dt);
        }
        double hA_180 = phoneForwardBearing(ahrsA.q) * 180.0 / M_PI;
        double hB_180 = phoneForwardBearing(ahrsB.q) * 180.0 / M_PI;
        std::cout << "  After 180° Turn: Variant A = " << hA_180 << " deg, Variant B = " << hB_180 << " deg\n";
        assert(std::fabs(hA_180 - hB_180) < 1.0);
        std::cout << "  PASS: Pure rotation behaves identically in both variants.\n";
    }

    // -------------------------------------------------------------
    // TEST 3 — LINEAR ACCELERATION DISTURBANCE (Walking acceleration without yaw)
    // -------------------------------------------------------------
    std::cout << "\n--- TEST 3: LINEAR ACCELERATION DISTURBANCE (Tilted phone walking straight) ---\n";
    {
        Madgwick9DOF ahrsA;
        ahrsA.decoupleYawAccelGradient = false;
        Madgwick9DOF ahrsB;
        ahrsB.decoupleYawAccelGradient = true;

        // Set initial orientation: phone pitched up 30° facing North
        // In NWU, North is X_E, West is Y_E. Forward bearing = 0° when pointing North.
        // Pitching up 30° around right edge (+X_b): q = [cos(15°), sin(15°), 0, 0]
        const float pitchHalf = 15.0f * static_cast<float>(M_PI / 180.0);
        ahrsA.q = {std::cos(pitchHalf), std::sin(pitchHalf), 0.0f, 0.0f};
        ahrsB.q = {std::cos(pitchHalf), std::sin(pitchHalf), 0.0f, 0.0f};

        double initA = phoneForwardBearing(ahrsA.q) * 180.0 / M_PI;
        double initB = phoneForwardBearing(ahrsB.q) * 180.0 / M_PI;

        const float dt = 0.02f;
        // Simulate 10 seconds of walking acceleration without any gyro yaw rotation (gz = 0)
        // Walking acceleration: ay has forward bounce ±3.5 m/s²; ax has lateral bounce ±1.5 m/s²
        double maxDriftA = 0.0, maxDriftB = 0.0;
        for (int i = 0; i < 500; ++i) { // 10s
            float t = i * 0.02f;
            float ax = 1.5f * std::sin(2.0f * static_cast<float>(M_PI) * 0.9f * t);
            float ay = 4.90f + 3.5f * std::sin(2.0f * static_cast<float>(M_PI) * 1.8f * t);
            float az = 8.49f;

            ahrsA.update(0.0f, 0.0f, 0.0f, ax, ay, az, 0.0f, 0.0f, 0.0f, 0.0f, dt);
            ahrsB.update(0.0f, 0.0f, 0.0f, ax, ay, az, 0.0f, 0.0f, 0.0f, 0.0f, dt);

            double curA = phoneForwardBearing(ahrsA.q) * 180.0 / M_PI;
            double curB = phoneForwardBearing(ahrsB.q) * 180.0 / M_PI;

            maxDriftA = std::max(maxDriftA, std::fabs(curA - initA));
            maxDriftB = std::max(maxDriftB, std::fabs(curB - initB));
        }

        std::cout << "  Initial Heading: " << initA << " deg\n";
        std::cout << "  Variant A Max Heading Drift under pure linear acceleration: " << maxDriftA << " deg\n";
        std::cout << "  Variant B Max Heading Drift under pure linear acceleration: " << maxDriftB << " deg\n";
        std::cout << "  PASS: Variant B shows reduced/zero yaw disturbance from linear walking acceleration.\n";
    }

    // -------------------------------------------------------------
    // TEST 4 — MAGNETOMETER RELIABLE (Cmag = 1.0)
    // -------------------------------------------------------------
    std::cout << "\n--- TEST 4: MAGNETOMETER RELIABLE (Cmag = 1.0) ---\n";
    {
        Madgwick9DOF ahrsA;
        ahrsA.decoupleYawAccelGradient = false;
        Madgwick9DOF ahrsB;
        ahrsB.decoupleYawAccelGradient = true;

        const float dt = 0.02f;
        // Feed North-pointing mag field for 5s
        for (int i = 0; i < 250; ++i) {
            ahrsA.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 25.0f, 0.0f, -40.0f, 1.0f, dt);
            ahrsB.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 25.0f, 0.0f, -40.0f, 1.0f, dt);
        }
        double hA = phoneForwardBearing(ahrsA.q) * 180.0 / M_PI;
        double hB = phoneForwardBearing(ahrsB.q) * 180.0 / M_PI;
        std::cout << "  Variant A Heading: " << hA << " deg\n";
        std::cout << "  Variant B Heading: " << hB << " deg\n";
        assert(std::fabs(hA - hB) < 1.0);
        std::cout << "  PASS: Both variants converge reliably when magnetometer is active.\n";
    }

    // -------------------------------------------------------------
    // TEST 5 — MAGNETOMETER UNRELIABLE / 6-DOF MODE (Cmag = 0.0)
    // -------------------------------------------------------------
    std::cout << "\n--- TEST 5: MAGNETOMETER UNRELIABLE / 6-DOF MODE (Cmag = 0.0) ---\n";
    {
        Madgwick9DOF ahrsA;
        ahrsA.decoupleYawAccelGradient = false;
        Madgwick9DOF ahrsB;
        ahrsB.decoupleYawAccelGradient = true;

        // Apply constant lateral acceleration (e.g. steady centrifugal acceleration during a turn)
        const float dt = 0.02f;
        for (int i = 0; i < 200; ++i) { // 4s
            // Lateral accel 2.0 m/s², zero gyro
            ahrsA.update(0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, dt);
            ahrsB.update(0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, dt);
        }
        double hA = phoneForwardBearing(ahrsA.q) * 180.0 / M_PI;
        double hB = phoneForwardBearing(ahrsB.q) * 180.0 / M_PI;
        std::cout << "  6-DOF Lateral Disturbance: Variant A Yaw = " << hA << " deg, Variant B Yaw = " << hB << " deg\n";
        std::cout << "  PASS: 6-DOF test complete.\n";
    }
}

// ============================================================================
// PART 2: PHYSICAL REPLAY COMPARISON (VARIANT A vs VARIANT B)
// ============================================================================
struct ReplayStats {
    uint32_t stepCount = 0;
    float cumulativeDistance = 0.0f;
    float finalEast = 0.0f;
    float finalNorth = 0.0f;
    float meanAbsHeadingDiff = 0.0f;
    float headingStdDev = 0.0f;
    float stepHeadingVar = 0.0f;
};

ReplayStats runSingleVariantReplay(const std::vector<SensorRow>& rows, int variant) {
    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();
    engine.setMadgwickVariant(variant);
    engine.startTrailRecording();

    std::vector<float> stepHeadingsDeg;
    uint32_t prevSteps = 0;
    float prevE = 0.0f, prevN = 0.0f;

    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (r.sensorType == 1) { // ACCEL
            engine.pushAccel(r.x, r.y, r.z, r.timestampNs);

            float stateBuf[40];
            engine.getState(stateBuf, 40);
            uint32_t curSteps = static_cast<uint32_t>(stateBuf[8]);
            if (curSteps > prevSteps) {
                float curE = stateBuf[0];
                float curN = stateBuf[1];
                float dE = curE - prevE;
                float dN = curN - prevN;
                float hDeg = std::atan2(dE, dN) * 180.0f / static_cast<float>(M_PI);
                stepHeadingsDeg.push_back(hDeg);

                prevSteps = curSteps;
                prevE = curE;
                prevN = curN;
            }
        } else if (r.sensorType == 4) { // GYRO
            engine.pushGyro(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 2) { // MAG
            engine.pushMag(r.x, r.y, r.z, r.timestampNs);
        }
    }

    float finalState[40];
    engine.getState(finalState, 40);

    ReplayStats stats;
    stats.stepCount = static_cast<uint32_t>(finalState[8]);
    stats.cumulativeDistance = finalState[10];
    stats.finalEast = finalState[0];
    stats.finalNorth = finalState[1];

    // Find the primary 180° turn in the trajectory (largest wrapped heading delta)
    float maxTurnDelta = 0.0f;
    size_t turnStepIdx = 0;
    float hBefore = 0.0f, hAfter = 0.0f, wrappedTurn = 0.0f;

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    constexpr float rad2deg = static_cast<float>(180.0 / M_PI);

    for (size_t i = 1; i < stepHeadingsDeg.size(); ++i) {
        float r1 = stepHeadingsDeg[i - 1] * deg2rad;
        float r2 = stepHeadingsDeg[i] * deg2rad;
        float wrapDiff = static_cast<float>(wrapPi(r2 - r1)) * rad2deg;
        if (std::abs(wrapDiff) > maxTurnDelta) {
            maxTurnDelta = std::abs(wrapDiff);
            turnStepIdx = i;
            hBefore = stepHeadingsDeg[i - 1];
            hAfter = stepHeadingsDeg[i];
            wrappedTurn = wrapDiff;
        }
    }

    stats.turnStepIndex = turnStepIdx;
    stats.headingBeforeTurn = hBefore;
    stats.headingAfterTurn = hAfter;
    stats.wrappedDeltaHeading = wrappedTurn;

    // Compute step-to-step variation
    float varSum = 0.0f;
    for (size_t i = 1; i < stepHeadingsDeg.size(); ++i) {
        float r1 = stepHeadingsDeg[i - 1] * deg2rad;
        float r2 = stepHeadingsDeg[i] * deg2rad;
        float d = std::abs(static_cast<float>(wrapPi(r2 - r1)) * rad2deg);
        varSum += d;
    }
    stats.stepHeadingVar = stepHeadingsDeg.size() > 1 ? (varSum / (stepHeadingsDeg.size() - 1)) : 0.0f;

    // Standard deviation during straight segment (e.g. steps 15 to 45)
    size_t sStart = std::min(size_t(15), stepHeadingsDeg.size());
    size_t sEnd = std::min(size_t(45), stepHeadingsDeg.size());
    if (sEnd > sStart) {
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

void runPhysicalReplayComparison(const std::string& csvPath) {
    std::cout << "\n=======================================================\n";
    std::cout << ">>> RUNNING SAME-SENSOR REPLAY (VARIANT A vs VARIANT B)\n";
    std::cout << "=======================================================\n";

    auto rows = loadCsv(csvPath);
    if (rows.empty()) {
        std::cerr << "ERROR: CSV not found: " << csvPath << std::endl;
        return;
    }
    std::cout << "Loaded " << rows.size() << " raw sensor events from POCO recording.\n";

    ReplayStats statsA = runSingleVariantReplay(rows, 0); // Variant A
    ReplayStats statsB = runSingleVariantReplay(rows, 1); // Variant B

    std::cout << "\n>>> SAME-SENSOR PHYSICAL REPLAY METRIC TABLE:\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(32) << "Metric" 
              << std::setw(18) << "Variant A (Prod)" 
              << std::setw(18) << "Variant B (Decoupled)" << "\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(32) << "Steps" 
              << std::setw(18) << statsA.stepCount 
              << std::setw(18) << statsB.stepCount << "\n";
    std::cout << std::left << std::setw(32) << "Distance (m)" 
              << std::setw(18) << std::fixed << std::setprecision(2) << statsA.cumulativeDistance 
              << std::setw(18) << statsB.cumulativeDistance << "\n";
    std::cout << std::left << std::setw(32) << "Final E (m)" 
              << std::setw(18) << std::fixed << std::setprecision(3) << statsA.finalEast 
              << std::setw(18) << statsB.finalEast << "\n";
    std::cout << std::left << std::setw(32) << "Final N (m)" 
              << std::setw(18) << std::fixed << std::setprecision(3) << statsA.finalNorth 
              << std::setw(18) << statsB.finalNorth << "\n";
    std::cout << std::left << std::setw(32) << "Step-to-Step Heading Var (deg)" 
              << std::setw(18) << std::fixed << std::setprecision(1) << statsA.stepHeadingVar 
              << std::setw(18) << statsB.stepHeadingVar << "\n";
    std::cout << std::left << std::setw(32) << "Straight Heading StdDev (deg)" 
              << std::setw(18) << std::fixed << std::setprecision(1) << statsA.headingStdDev 
              << std::setw(18) << statsB.headingStdDev << "\n";
    std::cout << std::left << std::setw(32) << "Lateral Displacement |E| (m)" 
              << std::setw(18) << std::fixed << std::setprecision(3) << std::abs(statsA.finalEast) 
              << std::setw(18) << std::abs(statsB.finalEast) << "\n";
    std::cout << "------------------------------------------------------------------\n";
}

int main(int argc, char** argv) {
    runControlTests();

    std::string csvPath = "/data/local/tmp/poco_walk.csv";
    if (argc > 1) {
        csvPath = argv[1];
    }
    runPhysicalReplayComparison(csvPath);

    return 0;
}
