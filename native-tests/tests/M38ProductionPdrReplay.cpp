#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <cassert>

#include "ahrs/NavigationEngine.hpp"
#include "pdr/PdrEngine.hpp"

struct SensorRow {
    int64_t timestampNs = 0;
    int sensorType = 0; // 1: Accel, 4: Gyro, 2: Mag
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

static std::vector<SensorRow> loadCsv(const std::string& path) {
    std::vector<SensorRow> rows;
    std::ifstream file(path);
    if (!file.is_open()) return rows;
    std::string line;
    if (!std::getline(file, line)) return rows; // header
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string item;
        SensorRow r;
        if (!std::getline(ss, item, ',')) continue;
        r.timestampNs = std::stoll(item);
        if (!std::getline(ss, item, ',')) continue;
        r.sensorType = std::stoi(item);
        if (!std::getline(ss, item, ',')) continue;
        r.x = std::stof(item);
        if (!std::getline(ss, item, ',')) continue;
        r.y = std::stof(item);
        if (!std::getline(ss, item, ',')) continue;
        r.z = std::stof(item);
        rows.push_back(r);
    }
    return rows;
}

struct EngineResult {
    uint32_t stepCount = 0;
    float distance = 0.0f;
    float finalE = 0.0f;
    float finalN = 0.0f;
    float netDisp = 0.0f;
    float travelHeadingDeg = 0.0f;
    float phoneHeadingDeg = 0.0f;
    size_t trailPoints = 0;
};

static EngineResult runEngine(const std::vector<SensorRow>& data, CourseEstimatorMode mode) {
    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();
    engine.setCourseMode(mode);
    engine.startTrailRecording();

    for (const auto& r : data) {
        if (r.sensorType == 4) {
            engine.pushGyro(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 2) {
            engine.pushMag(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 1) {
            engine.pushAccel(r.x, r.y, r.z, r.timestampNs);
        }
    }

    engine.stopTrailRecording();

    EngineResult res;
    res.stepCount = engine.getStepCount();
    res.distance = engine.getCumulativeDistance();
    res.finalE = engine.getEast();
    res.finalN = engine.getNorth();
    res.netDisp = std::hypot(res.finalE, res.finalN);
    res.travelHeadingDeg = engine.getTravelHeading() * 180.0f / 3.14159265f;
    res.phoneHeadingDeg = engine.getHeading() * 180.0f / 3.14159265f;
    res.trailPoints = engine.getTrailMemory().getPointCount();
    return res;
}

int main() {
    std::cout << "================================================================================" << std::endl;
    std::cout << "  M3.8 PRODUCTION NAVIGATION ENGINE A/B INTEGRATION REPLAY" << std::endl;
    std::cout << "  MODE A: PHONE_HEADING_BASELINE vs MODE B: CONSTRAINED_TRAVEL_COURSE" << std::endl;
    std::cout << "================================================================================" << std::endl;

    std::vector<std::pair<std::string, std::string>> scenarios = {
        {"SCENARIO A (Straight Walk)", "/data/local/tmp/m37_A_straight.csv"},
        {"SCENARIO B (Phone Yaw +-55°)", "/data/local/tmp/m37_B_phone_yaw.csv"},
        {"SCENARIO C (90° Turn)", "/data/local/tmp/m37_C_turn90.csv"},
        {"SCENARIO D (180° Reversal)", "/data/local/tmp/m37_D_reverse180.csv"},
        {"SCENARIO E (Stop-Rotate-Resume)", "/data/local/tmp/m37_E_stop_rotate_resume.csv"},
        {"SCENARIO F1 (Fixed +45° Yaw)", "/data/local/tmp/m37_F_phone_offset45.csv"},
        {"SCENARIO F2 (Fixed +90° Yaw)", "/data/local/tmp/m37_F_phone_offset90.csv"},
        {"SCENARIO G (Dangling Arm Carry)", "/data/local/tmp/m37_G_varied_posture.csv"},
        {"POCO Physical Walk (87 steps)", "/data/local/tmp/poco_walk.csv"}
    };

    for (const auto& sc : scenarios) {
        auto data = loadCsv(sc.second);
        if (data.empty()) {
            std::cout << "Skipping " << sc.first << " (file not found or empty)" << std::endl;
            continue;
        }

        EngineResult resA = runEngine(data, CourseEstimatorMode::PHONE_HEADING_BASELINE);
        EngineResult resB = runEngine(data, CourseEstimatorMode::CONSTRAINED_TRAVEL_COURSE);

        std::cout << "\n>>> " << sc.first << " (" << data.size() << " samples)" << std::endl;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        std::cout << std::left << std::setw(28) << "Metric" 
                  << std::setw(24) << "MODE A (Baseline)" 
                  << std::setw(24) << "MODE B (Constrained)" << std::endl;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
        std::cout << std::left << std::setw(28) << "1. Step Count" 
                  << std::setw(24) << resA.stepCount 
                  << std::setw(24) << resB.stepCount << std::endl;
        std::cout << std::left << std::setw(28) << "2. Cumulative Distance" 
                  << std::setw(24) << (std::to_string(resA.distance) + " m")
                  << std::setw(24) << (std::to_string(resB.distance) + " m") << std::endl;
        std::cout << std::left << std::setw(28) << "3. Final East (m)" 
                  << std::setw(24) << resA.finalE 
                  << std::setw(24) << resB.finalE << std::endl;
        std::cout << std::left << std::setw(28) << "4. Final North (m)" 
                  << std::setw(24) << resA.finalN 
                  << std::setw(24) << resB.finalN << std::endl;
        std::cout << std::left << std::setw(28) << "5. Net Displacement (m)" 
                  << std::setw(24) << resA.netDisp 
                  << std::setw(24) << resB.netDisp << std::endl;
        std::cout << std::left << std::setw(28) << "6. Travel Heading (deg)" 
                  << std::setw(24) << resA.travelHeadingDeg 
                  << std::setw(24) << resB.travelHeadingDeg << std::endl;
        std::cout << std::left << std::setw(28) << "7. Phone Heading (deg)" 
                  << std::setw(24) << resA.phoneHeadingDeg 
                  << std::setw(24) << resB.phoneHeadingDeg << std::endl;
        std::cout << std::left << std::setw(28) << "8. Trail Breadcrumbs" 
                  << std::setw(24) << resA.trailPoints 
                  << std::setw(24) << resB.trailPoints << std::endl;
        std::cout << "--------------------------------------------------------------------------------" << std::endl;
    }

    std::cout << "\n================================================================================" << std::endl;
    std::cout << "  M3.8 PRODUCTION INTEGRATION REPLAY COMPLETE — ALL RUNS EXECUTED VIA NAVIGATIONENGINE" << std::endl;
    std::cout << "================================================================================" << std::endl;
    return 0;
}
