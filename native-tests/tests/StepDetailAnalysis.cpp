#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include "ahrs/NavigationEngine.hpp"
#include "ahrs/MathTypes.hpp"

int main() {
    std::ifstream file("/data/local/tmp/poco_walk.csv");
    if (!file.is_open()) {
        std::cerr << "Cannot open poco_walk.csv\n";
        return 1;
    }
    std::string line;
    std::getline(file, line);
    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();
    engine.startTrailRecording();

    uint32_t prevSteps = 0;
    int64_t t0 = -1;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string item;
        int64_t t;
        int type;
        float x, y, z;
        std::getline(ss, item, ','); t = std::stoll(item);
        std::getline(ss, item, ','); type = std::stoi(item);
        std::getline(ss, item, ','); x = std::stof(item);
        std::getline(ss, item, ','); y = std::stof(item);
        std::getline(ss, item, ','); z = std::stof(item);

        if (t0 < 0) t0 = t;

        if (type == 1) {
            engine.pushAccel(x, y, z, t);
            float state[40];
            engine.getState(state, 40);
            uint32_t curSteps = static_cast<uint32_t>(state[8]);
            if (curSteps > prevSteps && curSteps <= 15) {
                Quaternion q = engine.getQuaternion();
                const double yN = 2.0 * (q.x * q.y - q.w * q.z);
                const double yW = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
                const double zN = 2.0 * (q.x * q.z + q.w * q.y);
                const double zW = 2.0 * (q.y * q.z - q.w * q.x);
                const double hA = std::hypot(yN, yW);
                const double hB = std::hypot(zN, zW);
                int axis = (hA >= hB) ? 0 : 1;
                double hDeg = phoneForwardBearing(q) * 180.0 / M_PI;
                std::cout << "Step " << std::setw(2) << curSteps 
                          << " at t=" << std::fixed << std::setprecision(2) << (t - t0)*1e-9 << "s"
                          << " | Axis: " << (axis == 0 ? "+Y_B" : "-Z_B")
                          << " | hA=" << hA << " hB=" << hB << " diff=" << (hA - hB)
                          << " | Heading: " << hDeg << " deg"
                          << " | q: [" << q.w << "," << q.x << "," << q.y << "," << q.z << "]"
                          << std::endl;
                prevSteps = curSteps;
            }
        } else if (type == 4) {
            engine.pushGyro(x, y, z, t);
        } else if (type == 2) {
            engine.pushMag(x, y, z, t);
        }
    }
    return 0;
}
