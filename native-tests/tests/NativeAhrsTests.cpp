#include "../../native/ahrs/MathTypes.hpp"
#include "../../native/ahrs/Madgwick9DOF.hpp"
#include "../../native/ahrs/MagneticReliability.hpp"
#include "../../native/ahrs/StationaryDetector.hpp"
#include "../../native/ahrs/NavigationEngine.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

static int g_failures = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            g_failures++; \
        } else { \
            std::cout << "PASS: " << msg << "\n"; \
        } \
    } while(0)

void testQuaternionNormalization() {
    Quaternion q{2.0f, -3.0f, 4.0f, -5.0f};
    Quaternion qNorm = normalizeQ(q);
    float norm = std::sqrt(qNorm.w*qNorm.w + qNorm.x*qNorm.x + qNorm.y*qNorm.y + qNorm.z*qNorm.z);
    TEST_ASSERT(std::abs(norm - 1.0f) < 1.0e-5f, "Quaternion normalization produces unit length");
}

void testGyroIntegration() {
    Madgwick9DOF ahrs;
    ahrs.beta = 0.0f; // Pure gyro integration, no gradient descent
    ahrs.q = {1.0f, 0.0f, 0.0f, 0.0f};

    // Integrate constant yaw angular velocity around z-axis: 1.0 rad/s for 1.0s at 50 Hz
    const float dt = 0.02f;
    const float gz = 1.0f; // 1 rad/s
    for (int i = 0; i < 50; ++i) {
        ahrs.update(0.0f, 0.0f, gz, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, dt);
    }

    // Expected angle: ~1.0 radian around z
    // For pure z rotation: q = [cos(theta/2), 0, 0, sin(theta/2)]
    const float expectedHalfAngle = 0.5f; // 1.0 / 2
    TEST_ASSERT(std::abs(ahrs.q.w - std::cos(expectedHalfAngle)) < 0.05f, "Gyro integration matches expected cos(theta/2)");
    TEST_ASSERT(std::abs(ahrs.q.z - std::sin(expectedHalfAngle)) < 0.05f, "Gyro integration matches expected sin(theta/2)");
}

void testStationarySensorHandlingAndBiasRecalibration() {
    StationaryDetector det;
    TEST_ASSERT(!det.isStationary(), "Initial stationary state is false");

    // Feed 40 stationary samples with small gyro bias: 0.03 rad/s on x
    const float biasX = 0.03f;
    for (int i = 0; i < 40; ++i) {
        det.update(0.0f, 0.0f, 9.81f, biasX, 0.0f, 0.0f);
    }

    TEST_ASSERT(det.isStationary(), "Stationary detector activates after stable window");
    TEST_ASSERT(det.gyroBias.x > 0.005f, "Gyro bias estimate converges toward persistent stationary bias");
}

void testLocalBaselineCalibration() {
    MagneticReliability magRel;
    Quaternion qLevel{1.0f, 0.0f, 0.0f, 0.0f};

    TEST_ASSERT(!magRel.isCalibrated(), "Initially uncalibrated (no hardcoded baseline)");
    TEST_ASSERT(magRel.getCalibrationStatus() == MagneticReliability::CalibrationStatus::UNAVAILABLE, "Initial status is UNAVAILABLE");

    // Feed 20 valid stationary samples: norm = sqrt(0^2 + 35^2 + 23.2^2) ≈ 42.0 µT
    // dip = acos(23.2 / 42.0) ≈ 0.987 rad
    for (int i = 0; i < 20; ++i) {
        magRel.update(0.0f, 35.0f, 23.2f, qLevel, /*isStationary=*/true);
    }

    TEST_ASSERT(magRel.isCalibrated(), "Calibrated after 20 stationary samples");
    TEST_ASSERT(magRel.getCalibrationStatus() == MagneticReliability::CalibrationStatus::CALIBRATED, "Status is CALIBRATED");
    TEST_ASSERT(std::abs(magRel.getB0() - 42.0f) < 0.2f, "Calibrated B0 matches local mean (~42.0 µT)");
    TEST_ASSERT(std::abs(magRel.getDip0() - 0.987f) < 0.05f, "Calibrated dip0 matches local mean (~0.987 rad)");
}

void testNominalFieldNearCalibratedB0() {
    MagneticReliability magRel;
    Quaternion qLevel{1.0f, 0.0f, 0.0f, 0.0f};

    // Calibrate with local field ~42.0 µT
    for (int i = 0; i < 20; ++i) {
        magRel.update(0.0f, 35.0f, 23.2f, qLevel, true);
    }
    TEST_ASSERT(magRel.isCalibrated(), "Calibrated successfully");

    // Feed nominal sample matching local baseline
    float cClean = magRel.update(0.0f, 35.0f, 23.2f, qLevel, false);
    TEST_ASSERT(cClean > 0.85f, "Nominal field near calibrated B0 produces high Cmag (> 0.85)");
}

void testDisturbedMagnitude() {
    MagneticReliability magRel;
    Quaternion qLevel{1.0f, 0.0f, 0.0f, 0.0f};

    // Calibrate with local baseline ~42.0 µT
    for (int i = 0; i < 20; ++i) {
        magRel.update(0.0f, 35.0f, 23.2f, qLevel, true);
    }

    // Severely disturbed magnitude sample (e.g. 100, 100, 100 -> norm ≈ 173 µT, >> 42 µT)
    float cDisturbed = magRel.update(100.0f, 100.0f, 100.0f, qLevel, false);
    TEST_ASSERT(cDisturbed < 0.05f, "Disturbed magnitude field is rejected (Cmag < 0.05)");
}

void testHighVariance() {
    MagneticReliability magRel;
    Quaternion qLevel{1.0f, 0.0f, 0.0f, 0.0f};

    // Calibrate baseline ~42.0 µT
    for (int i = 0; i < 20; ++i) {
        magRel.update(0.0f, 35.0f, 23.2f, qLevel, true);
    }

    // Inject fluctuating/oscillating field (std >> 3 µT)
    for (int i = 0; i < 15; ++i) {
        float yVal = (i % 2 == 0) ? 15.0f : 60.0f;
        magRel.update(0.0f, yVal, 23.2f, qLevel, false);
    }

    float cVar = magRel.getCmag();
    TEST_ASSERT(cVar < 0.20f, "High variance field de-weights magnetic correction (Cmag < 0.20)");
}

void testInsufficientCalibrationSamples() {
    MagneticReliability magRel;
    Quaternion qLevel{1.0f, 0.0f, 0.0f, 0.0f};

    // Feed only 5 stationary samples (< minCalibSamples = 20)
    for (int i = 0; i < 5; ++i) {
        float c = magRel.update(0.0f, 35.0f, 23.2f, qLevel, /*isStationary=*/true);
        TEST_ASSERT(c == 0.0f, "Cmag is 0.0 conservative fallback during insufficient calibration");
    }

    TEST_ASSERT(!magRel.isCalibrated(), "Insufficient samples leaves calibration unavailable");
    TEST_ASSERT(magRel.getCalibrationStatus() == MagneticReliability::CalibrationStatus::CALIBRATING, "Status is CALIBRATING");
    TEST_ASSERT(magRel.getCalibrationSampleCount() == 5, "Sample count is 5");

    // Device starts moving before calibration completes -> buffer resets
    magRel.update(0.0f, 35.0f, 23.2f, qLevel, /*isStationary=*/false);
    TEST_ASSERT(!magRel.isCalibrated(), "Still uncalibrated after interruption");
    TEST_ASSERT(magRel.getCalibrationStatus() == MagneticReliability::CalibrationStatus::UNAVAILABLE, "Status reset to UNAVAILABLE");
    TEST_ASSERT(magRel.getCalibrationSampleCount() == 0, "Buffer cleared after motion interruption");
    TEST_ASSERT(magRel.getCmag() == 0.0f, "Conservative fallback Cmag = 0.0 remains in effect");
}

void testTimestampOrderingAndGuards() {
    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();

    int64_t t = 1000000000LL;
    engine.pushAccel(0.0f, 0.0f, 9.81f, t);
    engine.pushGyro(0.0f, 0.0f, 0.0f, t);

    // Retrograde timestamp (dt < 0)
    engine.pushGyro(0.1f, 0.0f, 0.0f, t - 1000000LL);
    Quaternion qAfterRetrograde = engine.getQuaternion();
    TEST_ASSERT(!std::isnan(qAfterRetrograde.w) && !std::isnan(qAfterRetrograde.x), "Retrograde timestamp rejected safely without NaN");

    // Large time jump (> 0.25s)
    engine.pushGyro(0.1f, 0.0f, 0.0f, t + 1000000000LL); // +1.0s
    Quaternion qAfterJump = engine.getQuaternion();
    TEST_ASSERT(!std::isnan(qAfterJump.w), "Large timestamp jump handled safely");
}

void testZeroInvalidInputProtection() {
    Madgwick9DOF ahrs;
    ahrs.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);
    TEST_ASSERT(!std::isnan(ahrs.q.w) && !std::isnan(ahrs.q.x), "Zero sensor inputs handled without division by zero");
}

void testHeadingOutputSanity() {
    // In Earth NWU: x = North, y = West. For phone forward (+y_b) to point North (x_e),
    // phone is rotated 90° clockwise around Z: q = [cos(-45°), 0, 0, sin(-45°)] = [0.7071, 0, 0, -0.7071]
    Quaternion qNorth{0.7071068f, 0.0f, 0.0f, -0.7071068f};
    double headingNorth = phoneForwardBearing(qNorth);
    TEST_ASSERT(std::abs(headingNorth) < 0.01, "Phone forward (+y_b) facing North produces bearing ~0 rad");

    // Phone rotated further 90° clockwise (facing East in ENU: bearing = +π/2 = 1.5708 rad):
    // q = [cos(-90°), 0, 0, sin(-90°)] = [0, 0, 0, -1]
    Quaternion qEast{0.0f, 0.0f, 0.0f, -1.0f};
    double headingEast = phoneForwardBearing(qEast);
    TEST_ASSERT(std::abs(headingEast - (M_PI / 2.0)) < 0.01, "Phone facing East produces bearing ~+π/2 rad (clockwise)");
}

int main() {
    std::cout << "=== Running Milestone 2 Native AHRS Tests ===\n";
    testQuaternionNormalization();
    testGyroIntegration();
    testStationarySensorHandlingAndBiasRecalibration();
    testLocalBaselineCalibration();
    testNominalFieldNearCalibratedB0();
    testDisturbedMagnitude();
    testHighVariance();
    testInsufficientCalibrationSamples();
    testTimestampOrderingAndGuards();
    testZeroInvalidInputProtection();
    testHeadingOutputSanity();

    if (g_failures == 0) {
        std::cout << "\n>>> ALL M2 NATIVE TESTS PASSED! <<<\n";
        return 0;
    } else {
        std::cerr << "\n>>> " << g_failures << " TEST(S) FAILED! <<<\n";
        return 1;
    }
}
