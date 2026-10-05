#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <cassert>
#include <iomanip>

#include "ahrs/NavigationEngine.hpp"


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
    // Skip header
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

int main(int argc, char** argv) {
    std::string csvPath = "/data/local/tmp/poco_walk.csv";
    if (argc > 1) {
        csvPath = argv[1];
    }

    std::cout << "=======================================================" << std::endl;
    std::cout << "M4 PHYSICAL POCO TRAIL VALIDATION & REPLAY VERIFICATION" << std::endl;
    std::cout << "=======================================================" << std::endl;

    auto rows = loadCsv(csvPath);
    if (rows.empty()) {
        std::cerr << "ERROR: No sensor data loaded from " << csvPath << std::endl;
        return 1;
    }
    std::cout << "Loaded " << rows.size() << " raw sensor events from POCO walk recording." << std::endl;

    NavigationEngine& engine = NavigationEngine::instance();

    // -------------------------------------------------------------
    // PART A & B: JOURNEY 1 — Start journey and walk L-shaped route
    // -------------------------------------------------------------
    std::cout << "\n>>> Starting Journey 1 (Trail Recording)..." << std::endl;
    engine.reset();
    engine.startTrailRecording();

    float stateBuf[20];
    engine.getState(stateBuf, 20);
    std::cout << "Initial Journey 1 State: TrailState=" << static_cast<int>(stateBuf[17])
              << " | Points=" << static_cast<int>(stateBuf[16])
              << " | Steps=" << static_cast<int>(stateBuf[8])
              << " | Dist=" << stateBuf[19] << "m" << std::endl;
    assert(static_cast<int>(stateBuf[17]) == 1); // RECORDING
    assert(static_cast<int>(stateBuf[16]) == 1); // P0 anchor

    // Feed first 10,000 samples (~75 seconds: L-shaped physical walk)
    const size_t splitIndex = std::min(rows.size(), size_t(10000));
    for (size_t i = 0; i < splitIndex; ++i) {
        const auto& r = rows[i];
        if (r.sensorType == 1) { // ACCEL
            engine.pushAccel(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 4) { // GYRO
            engine.pushGyro(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 2) { // MAG
            engine.pushMag(r.x, r.y, r.z, r.timestampNs);
        }
    }

    engine.getState(stateBuf, 20);
    const int j1_steps = static_cast<int>(stateBuf[8]);
    const float j1_dist = stateBuf[10];
    const int j1_pts = static_cast<int>(stateBuf[16]);
    const float j1_duration = stateBuf[18];
    const float j1_east = stateBuf[0];
    const float j1_north = stateBuf[1];

    std::cout << "\n>>> Journey 1 Active Walk Finished:" << std::endl;
    std::cout << "    - Detected Steps: " << j1_steps << std::endl;
    std::cout << "    - Total PDR Distance: " << std::fixed << std::setprecision(2) << j1_dist << " m" << std::endl;
    std::cout << "    - Trail Points Stored: " << j1_pts << std::endl;
    std::cout << "    - Journey Duration: " << j1_duration << " s" << std::endl;
    std::cout << "    - Start Coordinate P0: (E=0.00, N=0.00)" << std::endl;
    std::cout << "    - End Coordinate Pn: (E=" << j1_east << ", N=" << j1_north << ")" << std::endl;

    assert(j1_pts == j1_steps + 1); // P0 + each accepted step

    // -------------------------------------------------------------
    // PART C & D: Stop recording & verify stored trail
    // -------------------------------------------------------------
    std::cout << "\n>>> Stopping Journey 1 Trail Recording..." << std::endl;
    bool stopOk = engine.stopTrailRecording();
    assert(stopOk);

    engine.getState(stateBuf, 20);
    std::cout << "Trail State after Stop: " << static_cast<int>(stateBuf[17]) << " (2 = RETURN_READY)" << std::endl;
    assert(static_cast<int>(stateBuf[17]) == 2); // RETURN_READY

    // Verify buffer copy of trail points
    std::vector<float> trailFloats(j1_pts * 4);
    int copied = engine.getTrailPoints(trailFloats.data(), j1_pts);
    assert(copied == j1_pts);

    std::cout << "Retrieved " << copied << " breadcrumbs from native memory:" << std::endl;
    std::cout << "   P0: E=" << trailFloats[0] << ", N=" << trailFloats[1] << ", Heading=" << trailFloats[2] << " rad, Dist=" << trailFloats[3] << "m" << std::endl;
    int midIdx = (j1_pts / 2) * 4;
    std::cout << "   P" << (j1_pts / 2) << ": E=" << trailFloats[midIdx] << ", N=" << trailFloats[midIdx+1] << ", Heading=" << trailFloats[midIdx+2] << " rad, Dist=" << trailFloats[midIdx+3] << "m" << std::endl;
    int lastIdx = (j1_pts - 1) * 4;
    std::cout << "   P" << (j1_pts - 1) << ": E=" << trailFloats[lastIdx] << ", N=" << trailFloats[lastIdx+1] << ", Heading=" << trailFloats[lastIdx+2] << " rad, Dist=" << trailFloats[lastIdx+3] << "m" << std::endl;

    // Verify ordering and monotonicity
    for (int i = 1; i < j1_pts; ++i) {
        float prevDist = trailFloats[(i - 1) * 4 + 3];
        float curDist = trailFloats[i * 4 + 3];
        assert(curDist >= prevDist); // Monotonic cumulative distance
    }
    std::cout << ">>> Trail ordering and cumulative distance monotonicity VERIFIED." << std::endl;

    // -------------------------------------------------------------
    // PART E & F: Start Journey 2 & Verify NO Stale Data Leakage
    // -------------------------------------------------------------
    std::cout << "\n>>> Starting Journey 2 (Explicit Reset & New Journey Start)..." << std::endl;
    engine.resetTrail();
    engine.startTrailRecording();

    engine.getState(stateBuf, 20);
    const int j2_init_steps = static_cast<int>(stateBuf[8]);
    const float j2_init_dist = stateBuf[10];
    const int j2_init_pts = static_cast<int>(stateBuf[16]);
    const float j2_init_east = stateBuf[0];
    const float j2_init_north = stateBuf[1];

    std::cout << "Journey 2 Initial State:" << std::endl;
    std::cout << "    - Step Count: " << j2_init_steps << " (Must be 0)" << std::endl;
    std::cout << "    - Cumulative Distance: " << j2_init_dist << " m (Must be 0.0)" << std::endl;
    std::cout << "    - Trail Point Count: " << j2_init_pts << " (Must be 1 for P0)" << std::endl;
    std::cout << "    - PDR Coordinates: (E=" << j2_init_east << ", N=" << j2_init_north << ") (Must be 0.0, 0.0)" << std::endl;

    assert(j2_init_steps == 0);
    assert(j2_init_dist == 0.0f);
    assert(j2_init_pts == 1);
    assert(std::abs(j2_init_east) < 1e-4f);
    assert(std::abs(j2_init_north) < 1e-4f);

    // Feed remaining samples for Journey 2
    for (size_t i = splitIndex; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (r.sensorType == 1) {
            engine.pushAccel(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 4) {
            engine.pushGyro(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 2) {
            engine.pushMag(r.x, r.y, r.z, r.timestampNs);
        }
    }

    engine.getState(stateBuf, 20);
    const int j2_steps = static_cast<int>(stateBuf[8]);
    const float j2_dist = stateBuf[10];
    const int j2_pts = static_cast<int>(stateBuf[16]);
    const float j2_duration = stateBuf[18];
    const float j2_east = stateBuf[0];
    const float j2_north = stateBuf[1];

    std::cout << "\n>>> Journey 2 Walk Finished:" << std::endl;
    std::cout << "    - Detected Steps: " << j2_steps << std::endl;
    std::cout << "    - Total PDR Distance: " << std::fixed << std::setprecision(2) << j2_dist << " m" << std::endl;
    std::cout << "    - Trail Points Stored: " << j2_pts << std::endl;
    std::cout << "    - Journey Duration: " << j2_duration << " s" << std::endl;
    std::cout << "    - Start Coordinate P0: (E=0.00, N=0.00)" << std::endl;
    std::cout << "    - End Coordinate Pn: (E=" << j2_east << ", N=" << j2_north << ")" << std::endl;

    assert(j2_pts == j2_steps + 1);
    assert(j2_pts != j1_pts); // Completely distinct journey

    engine.stopTrailRecording();
    engine.getState(stateBuf, 20);
    assert(static_cast<int>(stateBuf[17]) == 2); // RETURN_READY

    std::cout << "\n=======================================================" << std::endl;
    std::cout << "ALL PHYSICAL POCO TRAIL M4 CHECKS PASSED PERFECTLY!" << std::endl;
    std::cout << "=======================================================" << std::endl;
    return 0;
}
