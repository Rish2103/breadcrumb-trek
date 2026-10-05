#include <iostream>
#include <cmath>
#include <cassert>
#include <iomanip>
#include "ahrs/MathTypes.hpp"

#define ASSERT_NEAR(val, exp, tol, msg) \
    do { \
        if (std::abs((val) - (exp)) > (tol)) { \
            std::cerr << "FAIL: " << msg << " Expected ~" << (exp) << ", got " << (val) << "\n"; \
            return 1; \
        } else { \
            std::cout << "PASS: " << msg << " (" << (val) << ")\n"; \
        } \
    } while(0)

int main() {
    std::cout << "=======================================================\n";
    std::cout << ">>> RUNNING DETERMINISTIC CONTROL TESTS FOR BEARING\n";
    std::cout << "=======================================================\n";

    constexpr double deg2rad = M_PI / 180.0;
    constexpr double rad2deg = 180.0 / M_PI;

    // 1. Stable flat phone orientation facing North
    // In Earth NWU: x = North, y = West. For phone forward (+y_b) to point North (x_e),
    // phone is rotated 90° clockwise around Z: q = [cos(-45°), 0, 0, sin(-45°)] = [0.7071, 0, 0, -0.7071]
    {
        Quaternion qFlatNorth{0.7071068f, 0.0f, 0.0f, -0.7071068f};
        double b = phoneForwardBearing(qFlatNorth) * rad2deg;
        ASSERT_NEAR(b, 0.0, 0.1, "Test 1a: Flat phone facing North -> bearing 0°");
    }

    // Flat phone facing East (yaw 90° clockwise): q = [0, 0, 0, -1]
    {
        Quaternion qEast{0.0f, 0.0f, 0.0f, -1.0f};
        double b = phoneForwardBearing(qEast) * rad2deg;
        ASSERT_NEAR(b, 90.0, 0.1, "Test 1b: Flat phone facing East -> bearing +90°");
    }

    auto mul = [](const Quaternion& a, const Quaternion& b) -> Quaternion {
        return {
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
            a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
        };
    };

    const Quaternion qNorth{0.7071068f, 0.0f, 0.0f, -0.7071068f};

    // 2. Stable upright orientation facing North
    // Facing North, pitched 90° up around right axis +X_b
    {
        Quaternion qP90{static_cast<float>(std::cos(45.0 * deg2rad)), static_cast<float>(std::sin(45.0 * deg2rad)), 0.0f, 0.0f};
        Quaternion qUpright = mul(qNorth, qP90);
        double b = phoneForwardBearing(qUpright) * rad2deg;
        ASSERT_NEAR(b, 0.0, 0.1, "Test 2: Upright phone facing North -> bearing 0°");
    }

    // 3. Orientation near the hA/hB crossover (pitch = 45° facing North)
    {
        Quaternion qP45{static_cast<float>(std::cos(22.5 * deg2rad)), static_cast<float>(std::sin(22.5 * deg2rad)), 0.0f, 0.0f};
        Quaternion q45 = mul(qNorth, qP45);
        double b = phoneForwardBearing(q45) * rad2deg;
        ASSERT_NEAR(b, 0.0, 0.1, "Test 3: Exactly at 45° pitch crossover facing North -> bearing 0°");
    }

    // 4. Small pitch perturbations around the crossover (44.9° vs 45.1° facing North)
    {
        Quaternion qP44_9{static_cast<float>(std::cos(22.45 * deg2rad)), static_cast<float>(std::sin(22.45 * deg2rad)), 0.0f, 0.0f};
        Quaternion qP45_1{static_cast<float>(std::cos(22.55 * deg2rad)), static_cast<float>(std::sin(22.55 * deg2rad)), 0.0f, 0.0f};
        Quaternion q44_9 = mul(qNorth, qP44_9); // Pitch 44.9°: hA > hB (selects +Y_B)
        Quaternion q45_1 = mul(qNorth, qP45_1); // Pitch 45.1°: hB > hA (selects -Z_B)

        double b44_9 = phoneForwardBearing(q44_9) * rad2deg;
        double b45_1 = phoneForwardBearing(q45_1) * rad2deg;
        double delta = wrapPi((b45_1 - b44_9) * deg2rad) * rad2deg;

        std::cout << "  At 44.9° (uses +Y_B): Bearing = " << b44_9 << "°\n";
        std::cout << "  At 45.1° (uses -Z_B): Bearing = " << b45_1 << "°\n";
        std::cout << "  Delta across axis switch: " << delta << "°\n";

        ASSERT_NEAR(delta, 0.0, 0.01, "Test 4: Axis switch across 45° pitch causes 0.0° jump (NO 90° jump!)");
    }

    // Small pitch perturbations around crossover when facing East (Bearing = 90°)
    {
        // Yaw 90° then pitch 44.9° vs 45.1°
        // qYaw = [cos(45°), 0, 0, -sin(45°)]
        // qPitch = [cos(p/2), sin(p/2), 0, 0]
        // q = qYaw * qPitch
        Quaternion qYaw{0.7071068f, 0.0f, 0.0f, -0.7071068f};
        auto mul = [](const Quaternion& a, const Quaternion& b) -> Quaternion {
            return {
                a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
                a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
                a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
                a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
            };
        };

        Quaternion qP1{static_cast<float>(std::cos(22.45 * deg2rad)), static_cast<float>(std::sin(22.45 * deg2rad)), 0.0f, 0.0f};
        Quaternion qP2{static_cast<float>(std::cos(22.55 * deg2rad)), static_cast<float>(std::sin(22.55 * deg2rad)), 0.0f, 0.0f};
        Quaternion qE1 = mul(qYaw, qP1);
        Quaternion qE2 = mul(qYaw, qP2);

        double bE1 = phoneForwardBearing(qE1) * rad2deg;
        double bE2 = phoneForwardBearing(qE2) * rad2deg;
        double deltaE = wrapPi((bE2 - bE1) * deg2rad) * rad2deg;

        std::cout << "  East at 44.9° (+Y_B): Bearing = " << bE1 << "°\n";
        std::cout << "  East at 45.1° (-Z_B): Bearing = " << bE2 << "°\n";
        std::cout << "  Delta across axis switch: " << deltaE << "°\n";

        ASSERT_NEAR(deltaE, 0.0, 0.01, "Test 4b: East axis switch across 45° pitch causes 0.0° jump");
    }

    // 5. Genuine physical reorientation (smooth 0° -> 90° -> 180° rotation)
    {
        for (int angle = 0; angle <= 180; angle += 30) {
            float rad = static_cast<float>(angle * deg2rad);
            Quaternion qR{static_cast<float>(std::cos(-rad * 0.5f)), 0.0f, 0.0f, static_cast<float>(std::sin(-rad * 0.5f))};
            Quaternion qRotated = mul(qNorth, qR);
            double b = phoneForwardBearing(qRotated) * rad2deg;
            double diff = std::abs(wrapPi((b - static_cast<double>(angle)) * deg2rad)) * rad2deg;
            ASSERT_NEAR(diff, 0.0, 0.1, "Test 5: Legitimate rotation tracked correctly");
        }
    }

    std::cout << "\n>>> ALL CONTROL TESTS PASSED!\n";
    return 0;
}
