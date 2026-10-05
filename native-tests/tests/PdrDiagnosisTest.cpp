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

// Old PDR Engine (samples heading only at instantaneous peak)
class PdrEngineOld {
public:
    StepDetector stepDetector;
    StrideEstimator strideEstimator;

    bool update(float ax, float ay, float az, int64_t tNs, float currentHeadingRad, bool isStationary) {
        const float aMag = std::sqrt(ax * ax + ay * ay + az * az);
        StepDetector::Step step = stepDetector.push(aMag, tNs, isStationary);
        if (step.detected) {
            const float stride = strideEstimator.estimate(step.aMax, step.aMin);
            const float heading = currentHeadingRad; // Instantaneous heading only

            const float dE = stride * std::sin(heading);
            const float dN = stride * std::cos(heading);

            east_ += dE;
            north_ += dN;
            cumulativeDistance_ += stride;
            lastStride_ = stride;
            stepCount_++;
            lastHeading_ = heading;
            return true;
        }
        return false;
    }

    uint32_t getStepCount() const { return stepCount_; }
    float getEast() const { return east_; }
    float getNorth() const { return north_; }
    float getCumulativeDistance() const { return cumulativeDistance_; }
    float getLastHeading() const { return lastHeading_; }

    void reset() {
        stepDetector.reset();
        east_ = 0.0f;
        north_ = 0.0f;
        cumulativeDistance_ = 0.0f;
        lastStride_ = 0.0f;
        stepCount_ = 0;
        lastHeading_ = 0.0f;
    }

private:
    float east_ = 0.0f;
    float north_ = 0.0f;
    float cumulativeDistance_ = 0.0f;
    float lastStride_ = 0.0f;
    uint32_t stepCount_ = 0;
    float lastHeading_ = 0.0f;
};

// ============================================================================
// PART 1: DETERMINISTIC CIRCULAR MEAN TESTS (Req 9)
// ============================================================================
void runCircularMeanTests() {
    std::cout << "\n=======================================================\n";
    std::cout << ">>> RUNNING DETERMINISTIC CIRCULAR MEAN TESTS (Req 9)\n";
    std::cout << "=======================================================\n";

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    constexpr float rad2deg = static_cast<float>(180.0 / M_PI);

    // TEST A: 179°, -179° -> Expected approximately ±180°, NOT 0°
    {
        float anglesDeg[] = {179.0f, -179.0f};
        float anglesRad[] = {anglesDeg[0] * deg2rad, anglesDeg[1] * deg2rad};
        float meanRad = PdrEngine::circularMean(anglesRad, 2);
        float meanDeg = meanRad * rad2deg;
        std::cout << "TEST A (179°, -179°): Mean = " << meanDeg << "°\n";
        assert(std::fabs(std::fabs(meanDeg) - 180.0f) < 1.0f);
        std::cout << "  PASS: Circular mean correctly resolves to ~±180° (NOT 0°)\n";
    }

    // TEST B: 0°, 5°, -4°, 3°, -2° -> Expected approximately 0°
    {
        float anglesDeg[] = {0.0f, 5.0f, -4.0f, 3.0f, -2.0f};
        float anglesRad[5];
        for (int i = 0; i < 5; ++i) anglesRad[i] = anglesDeg[i] * deg2rad;
        float meanRad = PdrEngine::circularMean(anglesRad, 5);
        float meanDeg = meanRad * rad2deg;
        std::cout << "TEST B (0°, 5°, -4°, 3°, -2°): Mean = " << meanDeg << "°\n";
        assert(std::fabs(meanDeg) < 1.0f);
        std::cout << "  PASS: Circular mean correctly resolves to ~0°\n";
    }

    // TEST C: 90°, 92°, 88°, 91° -> Expected approximately 90°
    {
        float anglesDeg[] = {90.0f, 92.0f, 88.0f, 91.0f};
        float anglesRad[4];
        for (int i = 0; i < 4; ++i) anglesRad[i] = anglesDeg[i] * deg2rad;
        float meanRad = PdrEngine::circularMean(anglesRad, 4);
        float meanDeg = meanRad * rad2deg;
        std::cout << "TEST C (90°, 92°, 88°, 91°): Mean = " << meanDeg << "°\n";
        assert(std::fabs(meanDeg - 90.0f) < 1.0f);
        std::cout << "  PASS: Circular mean correctly resolves to ~90°\n";
    }

    // TEST D: 170°, 175°, -178°, -175° -> Expected approximately ±180°
    {
        float anglesDeg[] = {170.0f, 175.0f, -178.0f, -175.0f};
        float anglesRad[4];
        for (int i = 0; i < 4; ++i) anglesRad[i] = anglesDeg[i] * deg2rad;
        float meanRad = PdrEngine::circularMean(anglesRad, 4);
        float meanDeg = meanRad * rad2deg;
        std::cout << "TEST D (170°, 175°, -178°, -175°): Mean = " << meanDeg << "°\n";
        assert(std::fabs(std::fabs(meanDeg) - 180.0f) < 5.0f);
        std::cout << "  PASS: Circular mean correctly resolves across wrap to ~±180°\n";
    }
}

// ============================================================================
// PART 2: DETERMINISTIC INTEGRATION TEST: OSCILLATING HEADINGS (Req 11)
// ============================================================================
void runOscillatingHeadingIntegrationTest() {
    std::cout << "\n=======================================================\n";
    std::cout << ">>> RUNNING DETERMINISTIC OSCILLATION TEST (Req 11)\n";
    std::cout << "=======================================================\n";

    PdrEngine pdrNew;
    pdrNew.reset();
    PdrEngineOld pdrOld;
    pdrOld.reset();

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    int64_t tNs = 1000000000LL;
    const int64_t dtNs = 20000000LL; // 50 Hz

    // Settle baseline
    for (int i = 0; i < 50; ++i) {
        pdrNew.update(0.0f, 0.0f, 9.81f, tNs, 0.0f, false);
        pdrOld.update(0.0f, 0.0f, 9.81f, tNs, 0.0f, false);
        tNs += dtNs;
    }

    // Simulate 6 steps with arm-swing yaw oscillation around 0° (North)
    const float swingAmps[] = {45.0f, -45.0f, 35.0f, -35.0f, 50.0f, -50.0f};
    for (int stepIdx = 0; stepIdx < 6; ++stepIdx) {
        const float amp = swingAmps[stepIdx];
        for (int i = 0; i < 26; ++i) {
            float phase = static_cast<float>(i) / 26.0f;
            float aMag = 9.81f + 3.8f * std::sin(2.0f * static_cast<float>(M_PI) * phase);
            float hDeg = amp * std::cos(2.0f * static_cast<float>(M_PI) * phase);
            pdrNew.update(0.0f, 0.0f, aMag, tNs, hDeg * deg2rad, false);
            pdrOld.update(0.0f, 0.0f, aMag, tNs, hDeg * deg2rad, false);
            tNs += dtNs;
        }
    }

    std::cout << "Oscillating Headings Result:\n";
    std::cout << "  OLD (Peak-only): E = " << pdrOld.getEast() << " m, N = " << pdrOld.getNorth() << " m, Steps = " << pdrOld.getStepCount() << "\n";
    std::cout << "  NEW (Circular):  E = " << pdrNew.getEast() << " m, N = " << pdrNew.getNorth() << " m, Steps = " << pdrNew.getStepCount() << "\n";

    assert(pdrNew.getStepCount() >= 4);
    assert(std::abs(pdrNew.getEast()) < 0.25f);
    assert(pdrNew.getNorth() > 2.0f);
    std::cout << "  PASS: Circular mean maintains straight-line North tracking without lateral drift!\n";
}

// ============================================================================
// PART 3: PHYSICAL SENSOR REPLAY COMPARISON (OLD vs NEW) (Req 13, 14)
// ============================================================================
void runPhysicalReplayComparison(const std::string& csvPath) {
    std::cout << "\n=======================================================\n";
    std::cout << ">>> RUNNING PHYSICAL SENSOR REPLAY COMPARISON (Req 13, 14)\n";
    std::cout << "=======================================================\n";

    auto rows = loadCsv(csvPath);
    if (rows.empty()) {
        std::cerr << "ERROR: CSV not found: " << csvPath << std::endl;
        return;
    }
    std::cout << "Loaded " << rows.size() << " raw sensor events from POCO recording.\n";

    // Setup NavigationEngine with M3.1 PdrEngine
    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();
    engine.startTrailRecording();

    // Setup standalone Old PDR engine with identical AHRS heading feed
    PdrEngineOld oldPdr;
    oldPdr.reset();

    struct ComparisonStep {
        uint32_t stepIndex;
        int64_t tNs;
        float peakHeadingDeg;
        float averagedHeadingDeg;
        float deltaDeg;
        float stride;
        float old_dE, old_dN, old_cumE, old_cumN;
        float new_dE, new_dN, new_cumE, new_cumN;
    };

    std::vector<ComparisonStep> compSteps;
    uint32_t prevNewSteps = 0;
    float prevNewE = 0.0f, prevNewN = 0.0f;
    float prevOldE = 0.0f, prevOldN = 0.0f;

    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (r.sensorType == 1) { // ACCEL
            // Run AHRS forward bearing
            float stateBuf[40];
            engine.getState(stateBuf, 40);
            const float currentHeading = stateBuf[3]; // AHRS instantaneous forward bearing

            // Update Old PDR
            oldPdr.update(r.x, r.y, r.z, r.timestampNs, currentHeading, false);

            // Update NavigationEngine (uses New PdrEngine)
            engine.pushAccel(r.x, r.y, r.z, r.timestampNs);

            engine.getState(stateBuf, 40);
            uint32_t curNewSteps = static_cast<uint32_t>(stateBuf[8]);

            if (curNewSteps > prevNewSteps) {
                ComparisonStep cs;
                cs.stepIndex = curNewSteps;
                cs.tNs = r.timestampNs;
                cs.stride = stateBuf[9];

                // New PDR coordinates
                cs.new_cumE = stateBuf[0];
                cs.new_cumN = stateBuf[1];
                cs.new_dE = cs.new_cumE - prevNewE;
                cs.new_dN = cs.new_cumN - prevNewN;
                cs.averagedHeadingDeg = std::atan2(cs.new_dE, cs.new_dN) * 180.0f / static_cast<float>(M_PI);

                // Old PDR coordinates
                cs.old_cumE = oldPdr.getEast();
                cs.old_cumN = oldPdr.getNorth();
                cs.old_dE = cs.old_cumE - prevOldE;
                cs.old_dN = cs.old_cumN - prevOldN;
                cs.peakHeadingDeg = std::atan2(cs.old_dE, cs.old_dN) * 180.0f / static_cast<float>(M_PI);

                float diff = std::abs(cs.averagedHeadingDeg - cs.peakHeadingDeg);
                if (diff > 180.0f) diff = 360.0f - diff;
                cs.deltaDeg = diff;

                compSteps.push_back(cs);

                prevNewSteps = curNewSteps;
                prevNewE = cs.new_cumE;
                prevNewN = cs.new_cumN;
                prevOldE = cs.old_cumE;
                prevOldN = cs.old_cumN;
            }
        } else if (r.sensorType == 4) { // GYRO
            engine.pushGyro(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 2) { // MAG
            engine.pushMag(r.x, r.y, r.z, r.timestampNs);
        }
    }

    float finalState[40];
    engine.getState(finalState, 40);

    // Compute statistics
    float totalDiff = 0.0f;
    float peakVar = 0.0f;
    float avgVar = 0.0f;
    for (size_t i = 0; i < compSteps.size(); ++i) {
        totalDiff += compSteps[i].deltaDeg;
        if (i > 0) {
            float dp = std::abs(compSteps[i].peakHeadingDeg - compSteps[i-1].peakHeadingDeg);
            if (dp > 180.0f) dp = 360.0f - dp;
            peakVar += dp;

            float da = std::abs(compSteps[i].averagedHeadingDeg - compSteps[i-1].averagedHeadingDeg);
            if (da > 180.0f) da = 360.0f - da;
            avgVar += da;
        }
    }
    const float meanAbsDiff = compSteps.empty() ? 0.0f : (totalDiff / compSteps.size());
    const float meanPeakVar = compSteps.size() > 1 ? (peakVar / (compSteps.size() - 1)) : 0.0f;
    const float meanAvgVar  = compSteps.size() > 1 ? (avgVar / (compSteps.size() - 1)) : 0.0f;

    std::cout << "\n>>> COMPARISON METRICS SUMMARY TABLE (Req 13, 14):\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(32) << "Metric" 
              << std::setw(18) << "Old (Peak-Only)" 
              << std::setw(18) << "New (Circular-Mean)" << "\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(32) << "Steps" 
              << std::setw(18) << oldPdr.getStepCount() 
              << std::setw(18) << static_cast<int>(finalState[8]) << "\n";
    std::cout << std::left << std::setw(32) << "Distance (m)" 
              << std::setw(18) << std::fixed << std::setprecision(2) << oldPdr.getCumulativeDistance() 
              << std::setw(18) << finalState[10] << "\n";
    std::cout << std::left << std::setw(32) << "Final E (m)" 
              << std::setw(18) << std::fixed << std::setprecision(3) << oldPdr.getEast() 
              << std::setw(18) << finalState[0] << "\n";
    std::cout << std::left << std::setw(32) << "Final N (m)" 
              << std::setw(18) << std::fixed << std::setprecision(3) << oldPdr.getNorth() 
              << std::setw(18) << finalState[1] << "\n";
    std::cout << std::left << std::setw(32) << "Step-to-Step Heading Var (deg)" 
              << std::setw(18) << std::fixed << std::setprecision(1) << meanPeakVar 
              << std::setw(18) << meanAvgVar << "\n";
    std::cout << std::left << std::setw(32) << "Mean |H_avg - H_peak| (deg)" 
              << std::setw(18) << "N/A" 
              << std::setw(18) << std::fixed << std::setprecision(2) << meanAbsDiff << "\n";
    std::cout << std::left << std::setw(32) << "Lateral Displacement |E| (m)" 
              << std::setw(18) << std::fixed << std::setprecision(3) << std::abs(oldPdr.getEast()) 
              << std::setw(18) << std::abs(finalState[0]) << "\n";
    std::cout << "------------------------------------------------------------------\n";

    std::cout << "\nFirst 10 steps comparison detail:\n";
    std::cout << std::left << std::setw(6) << "Step"
              << std::setw(14) << "H_peak(deg)"
              << std::setw(14) << "H_avg(deg)"
              << std::setw(12) << "Delta(deg)"
              << std::setw(12) << "Old_CumE"
              << std::setw(12) << "New_CumE"
              << std::setw(12) << "Old_CumN"
              << std::setw(12) << "New_CumN" << "\n";
    std::cout << std::string(88, '-') << "\n";
    for (size_t i = 0; i < std::min(compSteps.size(), size_t(10)); ++i) {
        const auto& cs = compSteps[i];
        std::cout << std::left << std::setw(6) << cs.stepIndex
                  << std::setw(14) << std::fixed << std::setprecision(1) << cs.peakHeadingDeg
                  << std::setw(14) << std::fixed << std::setprecision(1) << cs.averagedHeadingDeg
                  << std::setw(12) << std::fixed << std::setprecision(1) << cs.deltaDeg
                  << std::setw(12) << std::fixed << std::setprecision(2) << cs.old_cumE
                  << std::setw(12) << std::fixed << std::setprecision(2) << cs.new_cumE
                  << std::setw(12) << std::fixed << std::setprecision(2) << cs.old_cumN
                  << std::setw(12) << std::fixed << std::setprecision(2) << cs.new_cumN << "\n";
    }
}

int main(int argc, char** argv) {
    runCircularMeanTests();
    runOscillatingHeadingIntegrationTest();

    std::string csvPath = "/data/local/tmp/poco_walk.csv";
    if (argc > 1) {
        csvPath = argv[1];
    }
    runPhysicalReplayComparison(csvPath);

    return 0;
}
