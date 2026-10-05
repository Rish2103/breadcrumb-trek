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

struct SampleDiagnostic {
    int64_t tNs;
    double hA;
    double hB;
    double diff; // hA - hB
    int axis; // 0 = +Y_B, 1 = -Z_B
    double fN;
    double fW;
    double headingRad;
    double headingDeg;
    Quaternion q;
    bool axisSwitched;
    double headingDeltaDeg;
};

int main(int argc, char** argv) {
    std::string csvPath = "/data/local/tmp/poco_walk.csv";
    if (argc > 1) {
        csvPath = argv[1];
    }

    auto rows = loadCsv(csvPath);
    if (rows.empty()) {
        std::cerr << "Could not load " << csvPath << std::endl;
        return 1;
    }
    std::cout << "Loaded " << rows.size() << " rows from " << csvPath << std::endl;

    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();
    engine.startTrailRecording();

    std::vector<SampleDiagnostic> samples;
    samples.reserve(rows.size());

    int prevAxis = -1;
    double prevHeading = 0.0;
    bool hasPrev = false;

    for (const auto& r : rows) {
        if (r.sensorType == 1) {
            engine.pushAccel(r.x, r.y, r.z, r.timestampNs);
        } else if (r.sensorType == 4) {
            engine.pushGyro(r.x, r.y, r.z, r.timestampNs);

            // Directly inspect quaternion after AHRS gyro update
            Quaternion q = engine.getQuaternion();
            const double yN = 2.0 * (q.x * q.y - q.w * q.z);
            const double yW = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
            const double zN = 2.0 * (q.x * q.z + q.w * q.y);
            const double zW = 2.0 * (q.y * q.z - q.w * q.x);

            const double hA = std::hypot(yN, yW); // +y_b horizontal extent
            const double hB = std::hypot(zN, zW); // -z_b horizontal extent

            int axis = (hA >= hB) ? 0 : 1;
            double fN = (axis == 0) ? yN : -zN;
            double fW = (axis == 0) ? yW : -zW;
            double headingRad = wrapPi(std::atan2(-fW, fN));
            double headingDeg = headingRad * 180.0 / M_PI;

            bool axisSwitched = false;
            double deltaDeg = 0.0;
            if (hasPrev) {
                if (axis != prevAxis) {
                    axisSwitched = true;
                }
                deltaDeg = wrapPi(headingRad - prevHeading) * 180.0 / M_PI;
            }

            samples.push_back({
                r.timestampNs,
                hA,
                hB,
                hA - hB,
                axis,
                fN,
                fW,
                headingRad,
                headingDeg,
                q,
                axisSwitched,
                deltaDeg
            });

            prevAxis = axis;
            prevHeading = headingRad;
            hasPrev = true;
        } else if (r.sensorType == 2) {
            engine.pushMag(r.x, r.y, r.z, r.timestampNs);
        }
    }

    if (samples.empty()) {
        std::cerr << "No AHRS samples generated.\n";
        return 1;
    }

    // PHASE 2 STATISTICS
    size_t totalSamples = samples.size();
    size_t countY = 0;
    size_t countZ = 0;
    size_t switchCount = 0;

    double minAbsDiff = 1e9;
    double sumAbsDiff = 0.0;

    int threshCount001 = 0;
    int threshCount002 = 0;
    int threshCount005 = 0;
    int threshCount010 = 0;

    for (const auto& s : samples) {
        if (s.axis == 0) countY++;
        else countZ++;

        if (s.axisSwitched) switchCount++;

        double absDiff = std::abs(s.diff);
        if (absDiff < minAbsDiff) minAbsDiff = absDiff;
        sumAbsDiff += absDiff;

        if (absDiff < 0.01) threshCount001++;
        if (absDiff < 0.02) threshCount002++;
        if (absDiff < 0.05) threshCount005++;
        if (absDiff < 0.10) threshCount010++;
    }

    double durationSec = (samples.back().tNs - samples.front().tNs) * 1e-9;
    double switchesPerSec = durationSec > 0 ? (switchCount / durationSec) : 0.0;
    double meanAbsDiff = sumAbsDiff / totalSamples;

    std::cout << "\n=======================================================\n";
    std::cout << ">>> PHASE 1 & 2: AXIS SWITCH DIAGNOSTICS\n";
    std::cout << "=======================================================\n";
    std::cout << "Total AHRS samples:    " << totalSamples << "\n";
    std::cout << "Duration (sec):        " << std::fixed << std::setprecision(2) << durationSec << " s\n";
    std::cout << "+Y_B selections:       " << countY << " (" << std::setprecision(1) << (countY * 100.0 / totalSamples) << "%)\n";
    std::cout << "-Z_B selections:       " << countZ << " (" << std::setprecision(1) << (countZ * 100.0 / totalSamples) << "%)\n";
    std::cout << "Total axis switches:   " << switchCount << "\n";
    std::cout << "Switches/sec:          " << std::setprecision(3) << switchesPerSec << "\n";
    std::cout << "Min |hA - hB|:         " << std::setprecision(6) << minAbsDiff << "\n";
    std::cout << "Mean |hA - hB|:        " << std::setprecision(6) << meanAbsDiff << "\n";
    std::cout << "|hA - hB| < 0.01:      " << threshCount001 << " (" << (threshCount001 * 100.0 / totalSamples) << "%)\n";
    std::cout << "|hA - hB| < 0.02:      " << threshCount002 << " (" << (threshCount002 * 100.0 / totalSamples) << "%)\n";
    std::cout << "|hA - hB| < 0.05:      " << threshCount005 << " (" << (threshCount005 * 100.0 / totalSamples) << "%)\n";
    std::cout << "|hA - hB| < 0.10:      " << threshCount010 << " (" << (threshCount010 * 100.0 / totalSamples) << "%)\n";

    // PHASE 3: CORRELATE AXIS SWITCHES WITH HEADING JUMPS
    int switchesAbove30 = 0;
    int switchesAbove60 = 0;
    int switchesNear90 = 0; // 75° to 105°

    int nonSwitchesAbove30 = 0;
    int nonSwitchesAbove60 = 0;
    int nonSwitchesNear90 = 0;

    double maxSwitchJump = 0.0;
    double maxNonSwitchJump = 0.0;

    std::vector<SampleDiagnostic> switchEvents;

    for (size_t i = 1; i < samples.size(); ++i) {
        const auto& s = samples[i];
        double absDelta = std::abs(s.headingDeltaDeg);

        if (s.axisSwitched) {
            switchEvents.push_back(s);
            if (absDelta > maxSwitchJump) maxSwitchJump = absDelta;
            if (absDelta > 30.0) switchesAbove30++;
            if (absDelta > 60.0) switchesAbove60++;
            if (absDelta >= 75.0 && absDelta <= 105.0) switchesNear90++;
        } else {
            if (absDelta > maxNonSwitchJump) maxNonSwitchJump = absDelta;
            if (absDelta > 30.0) nonSwitchesAbove30++;
            if (absDelta > 60.0) nonSwitchesAbove60++;
            if (absDelta >= 75.0 && absDelta <= 105.0) nonSwitchesNear90++;
        }
    }

    std::cout << "\n=======================================================\n";
    std::cout << ">>> PHASE 3: CORRELATION WITH HEADING JUMPS\n";
    std::cout << "=======================================================\n";
    std::cout << "Max heading jump on axis switch:      " << std::setprecision(2) << maxSwitchJump << " deg\n";
    std::cout << "Max heading jump WITHOUT axis switch:  " << std::setprecision(2) << maxNonSwitchJump << " deg\n";
    std::cout << "Axis switches with delta > 30 deg:     " << switchesAbove30 << " / " << switchCount << "\n";
    std::cout << "Axis switches with delta > 60 deg:     " << switchesAbove60 << " / " << switchCount << "\n";
    std::cout << "Axis switches with delta ~90 deg (75-105): " << switchesNear90 << " / " << switchCount << "\n";
    std::cout << "\nFor comparison (samples WITHOUT axis switch):\n";
    std::cout << "Non-switches with delta > 30 deg:     " << nonSwitchesAbove30 << " / " << (totalSamples - switchCount) << "\n";
    std::cout << "Non-switches with delta > 60 deg:     " << nonSwitchesAbove60 << " / " << (totalSamples - switchCount) << "\n";
    std::cout << "Non-switches with delta ~90 deg:      " << nonSwitchesNear90 << " / " << (totalSamples - switchCount) << "\n";

    std::cout << "\n--- Representative Axis Switch Events (First 10) ---\n";
    int printed = 0;
    for (size_t i = 1; i < samples.size() && printed < 10; ++i) {
        const auto& s = samples[i];
        if (s.axisSwitched) {
            const auto& prev = samples[i-1];
            std::cout << "  Switch #" << (printed + 1)
                      << " at t=" << std::setprecision(2) << ((s.tNs - samples[0].tNs)*1e-9) << "s: "
                      << (prev.axis == 0 ? "+Y_B" : "-Z_B") << " -> " << (s.axis == 0 ? "+Y_B" : "-Z_B")
                      << " | hA=" << std::setprecision(3) << s.hA << " hB=" << s.hB << " (diff=" << s.diff << ")"
                      << " | Heading: " << std::setprecision(1) << prev.headingDeg << "° -> " << s.headingDeg << "°"
                      << " | Delta: " << s.headingDeltaDeg << "°\n";
            printed++;
        }
    }

    return 0;
}
