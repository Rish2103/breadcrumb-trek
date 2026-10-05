#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
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

// Function to compute bearing with optional hysteresis margin
double computeBearingWithHysteresis(const Quaternion& q, int& currentAxis, double margin) {
    const double yN = 2.0 * (q.x * q.y - q.w * q.z);
    const double yW = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    const double zN = 2.0 * (q.x * q.z + q.w * q.y);
    const double zW = 2.0 * (q.y * q.z - q.w * q.x);

    const double hA = std::hypot(yN, yW);
    const double hB = std::hypot(zN, zW);

    if (margin <= 0.0) {
        // Variant A: instantaneous
        currentAxis = (hA >= hB) ? 0 : 1;
    } else {
        // Variant B: hysteresis locking
        if (currentAxis == 0) { // +Y_B
            if (hB > hA + margin) {
                currentAxis = 1;
            }
        } else if (currentAxis == 1) { // -Z_B
            if (hA > hB + margin) {
                currentAxis = 0;
            }
        } else {
            currentAxis = (hA >= hB) ? 0 : 1;
        }
    }

    double fN = (currentAxis == 0) ? yN : -zN;
    double fW = (currentAxis == 0) ? yW : -zW;
    return wrapPi(std::atan2(-fW, fN));
}

struct RunResult {
    int axisSwitches = 0;
    uint32_t stepCount = 0;
    float cumulativeDistance = 0.0f;
    float finalE = 0.0f;
    float finalN = 0.0f;
    float stepHeadingVar = 0.0f;
    float headingStdDev = 0.0f;
};

RunResult runSimulation(const std::vector<SensorRow>& rows, double margin) {
    Madgwick9DOF ahrs;
    MagneticReliability magRel;
    StationaryDetector statDet;
    PdrEngine pdr;

    int currentAxis = -1;
    int prevAxis = -1;
    int axisSwitches = 0;

    int64_t lastAccelT = 0;
    int64_t lastGyroT = 0;
    int64_t lastMagT = 0;
    Vec3 latestAccel{0,0,0};
    Vec3 latestMag{0,0,0};
    bool init = false;

    std::vector<float> stepHeadingsDeg;
    uint32_t prevSteps = 0;
    float prevE = 0.0f, prevN = 0.0f;

    for (const auto& r : rows) {
        if (r.sensorType == 1) { // ACCEL
            latestAccel = {r.x, r.y, r.z};
            lastAccelT = r.timestampNs;

            double bearing = computeBearingWithHysteresis(ahrs.q, currentAxis, margin);
            if (prevAxis != -1 && currentAxis != prevAxis) {
                axisSwitches++;
            }
            prevAxis = currentAxis;

            bool isStat = statDet.isStationary();
            bool stepDetected = pdr.update(r.x, r.y, r.z, r.timestampNs, static_cast<float>(bearing), isStat);
            if (stepDetected) {
                float curE = pdr.getEast();
                float curN = pdr.getNorth();
                float dE = curE - prevE;
                float dN = curN - prevN;
                float hDeg = std::atan2(dE, dN) * 180.0f / static_cast<float>(M_PI);
                stepHeadingsDeg.push_back(hDeg);
                prevE = curE;
                prevN = curN;
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

            computeBearingWithHysteresis(ahrs.q, currentAxis, margin);
            if (prevAxis != -1 && currentAxis != prevAxis) {
                axisSwitches++;
            }
            prevAxis = currentAxis;
        } else if (r.sensorType == 2) { // MAG
            latestMag = {r.x, r.y, r.z};
            lastMagT = r.timestampNs;
            magRel.update(r.x, r.y, r.z, ahrs.q, statDet.isStationary());
        }
    }

    RunResult res;
    res.axisSwitches = axisSwitches;
    res.stepCount = pdr.getStepCount();
    res.cumulativeDistance = pdr.getCumulativeDistance();
    res.finalE = pdr.getEast();
    res.finalN = pdr.getNorth();

    // Step heading variation
    float varSum = 0.0f;
    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    constexpr float rad2deg = static_cast<float>(180.0 / M_PI);
    for (size_t i = 1; i < stepHeadingsDeg.size(); ++i) {
        float r1 = stepHeadingsDeg[i - 1] * deg2rad;
        float r2 = stepHeadingsDeg[i] * deg2rad;
        float d = std::abs(static_cast<float>(wrapPi(r2 - r1)) * rad2deg);
        varSum += d;
    }
    res.stepHeadingVar = stepHeadingsDeg.size() > 1 ? (varSum / (stepHeadingsDeg.size() - 1)) : 0.0f;

    // Heading stddev
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
        res.headingStdDev = std::sqrt(varH / (sEnd - sStart));
    }

    return res;
}

int main() {
    auto rows = loadCsv("/data/local/tmp/poco_walk.csv");
    if (rows.empty()) {
        std::cerr << "Cannot open CSV\n";
        return 1;
    }

    RunResult varA = runSimulation(rows, 0.0);   // Variant A (Production, margin=0.0)
    RunResult varB = runSimulation(rows, 0.05);  // Variant B (Hysteresis margin=0.05)
    RunResult varC = runSimulation(rows, 0.10);  // Variant B (Hysteresis margin=0.10)

    std::cout << "\n=======================================================\n";
    std::cout << ">>> M3.3 A/B COMPARISON TABLE: REPLAY OF POCO_WALK.CSV\n";
    std::cout << "=======================================================\n";
    std::cout << std::left << std::setw(28) << "Metric"
              << std::setw(20) << "Variant A (Prod)"
              << std::setw(20) << "Variant B (M=0.05)"
              << std::setw(20) << "Variant B (M=0.10)" << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(28) << "Axis Switches"
              << std::setw(20) << varA.axisSwitches
              << std::setw(20) << varB.axisSwitches
              << std::setw(20) << varC.axisSwitches << "\n";
    std::cout << std::left << std::setw(28) << "Step Count"
              << std::setw(20) << varA.stepCount
              << std::setw(20) << varB.stepCount
              << std::setw(20) << varC.stepCount << "\n";
    std::cout << std::left << std::setw(28) << "Cumulative Distance (m)"
              << std::setw(20) << std::fixed << std::setprecision(2) << varA.cumulativeDistance
              << std::setw(20) << varB.cumulativeDistance
              << std::setw(20) << varC.cumulativeDistance << "\n";
    std::cout << std::left << std::setw(28) << "Final E (m)"
              << std::setw(20) << std::fixed << std::setprecision(3) << varA.finalE
              << std::setw(20) << varB.finalE
              << std::setw(20) << varC.finalE << "\n";
    std::cout << std::left << std::setw(28) << "Final N (m)"
              << std::setw(20) << std::fixed << std::setprecision(3) << varA.finalN
              << std::setw(20) << varB.finalN
              << std::setw(20) << varC.finalN << "\n";
    std::cout << std::left << std::setw(28) << "Lateral Drift |E| (m)"
              << std::setw(20) << std::fixed << std::setprecision(3) << std::abs(varA.finalE)
              << std::setw(20) << std::abs(varB.finalE)
              << std::setw(20) << std::abs(varC.finalE) << "\n";
    std::cout << std::left << std::setw(28) << "Step Heading Var (deg)"
              << std::setw(20) << std::fixed << std::setprecision(1) << varA.stepHeadingVar
              << std::setw(20) << varB.stepHeadingVar
              << std::setw(20) << varC.stepHeadingVar << "\n";
    std::cout << std::left << std::setw(28) << "Straight Heading StdDev"
              << std::setw(20) << std::fixed << std::setprecision(1) << varA.headingStdDev
              << std::setw(20) << varB.headingStdDev
              << std::setw(20) << varC.headingStdDev << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    return 0;
}
