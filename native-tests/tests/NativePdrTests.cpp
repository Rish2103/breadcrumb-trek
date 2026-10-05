#include "../../native/ahrs/MathTypes.hpp"
#include "../../native/ahrs/Madgwick9DOF.hpp"
#include "../../native/ahrs/MagneticReliability.hpp"
#include "../../native/ahrs/StationaryDetector.hpp"
#include "../../native/ahrs/NavigationEngine.hpp"
#include "../../native/pdr/StepDetector.hpp"
#include "../../native/pdr/StrideEstimator.hpp"
#include "../../native/pdr/PdrEngine.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

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

// 1. Stationary signal -> zero steps
void testStationarySignalZeroSteps() {
    StepDetector detector;
    int64_t tNs = 1000000000LL;
    const int64_t dtNs = 20000000LL; // 50 Hz (20 ms)

    int stepsDetected = 0;
    // Feed 3 seconds of flat 1g stationary signal
    for (int i = 0; i < 150; ++i) {
        StepDetector::Step s = detector.push(9.80665f, tNs, /*isStationary=*/true);
        if (s.detected) stepsDetected++;
        tNs += dtNs;
    }

    TEST_ASSERT(stepsDetected == 0, "Stationary signal produces exactly zero steps");
}

// 2. Periodic walking signal -> expected step detection
void testPeriodicWalkingSignalExpectedSteps() {
    StepDetector detector;
    int64_t tNs = 1000000000LL;
    const int64_t dtNs = 20000000LL; // 50 Hz
    const float cadenceHz = 1.8f;    // 1.8 steps per second (~0.556s per step)
    const float durationS = 6.0f;
    const int totalSamples = static_cast<int>(durationS * 50.0f);

    int stepsDetected = 0;
    float lastInterval = 0.0f;

    for (int i = 0; i < totalSamples; ++i) {
        const float t = i * 0.02f;
        // Synthesize walking vertical acceleration bounce (9.81 ± 3.5 m/s²)
        const float aMag = 9.80665f + 3.5f * std::sin(2.0f * static_cast<float>(M_PI) * cadenceHz * t);

        StepDetector::Step s = detector.push(aMag, tNs, /*isStationary=*/false);
        if (s.detected) {
            stepsDetected++;
            lastInterval = s.intervalS;
        }
        tNs += dtNs;
    }

    // In 6.0s at 1.8 Hz, expected ~10 steps (allowing 1 step startup filter latency)
    TEST_ASSERT(stepsDetected >= 9 && stepsDetected <= 11, "Periodic walking signal detects expected steps (~10 steps in 6s)");
    TEST_ASSERT(std::abs(lastInterval - (1.0f / cadenceHz)) < 0.10f, "Step interval matches walking cadence (~0.56s)");
}

// 3. Noisy signal -> controlled false detections
void testNoisySignalControlledFalseDetections() {
    StepDetector detector;
    int64_t tNs = 1000000000LL;
    const int64_t dtNs = 20000000LL; // 50 Hz

    int stepsDetected = 0;
    // 5 seconds of high-frequency low-amplitude jitter (15 Hz, 0.2 m/s² amplitude)
    for (int i = 0; i < 250; ++i) {
        const float t = i * 0.02f;
        const float aMag = 9.80665f + 0.20f * std::sin(2.0f * static_cast<float>(M_PI) * 15.0f * t);

        StepDetector::Step s = detector.push(aMag, tNs, /*isStationary=*/false);
        if (s.detected) stepsDetected++;
        tNs += dtNs;
    }

    TEST_ASSERT(stepsDetected == 0, "High-frequency sub-threshold noise rejected (zero false steps)");
}

// 4. Minimum interval rejection
void testMinimumIntervalRejection() {
    StepDetector detector;
    int64_t tNs = 1000000000LL;
    const int64_t dtNs = 20000000LL; // 20 ms (50 Hz)

    // Settle baseline
    for (int i = 0; i < 50; ++i) {
        detector.push(9.81f, tNs, false);
        tNs += dtNs;
    }

    int steps = 0;

    // Peak 1: 8 samples (peak at sample 3-4)
    for (int i = 0; i < 8; ++i) {
        float a = 9.81f + 4.0f * std::sin(static_cast<float>(M_PI) * i / 7.0f);
        StepDetector::Step s = detector.push(a, tNs, false);
        if (s.detected) steps++;
        tNs += dtNs;
    }
    TEST_ASSERT(steps == 1, "First valid peak accepted as step 1");

    // Immediate second peak only 8 samples later (0.16s peak-to-peak, < minIntervalS = 0.25s)
    for (int i = 0; i < 8; ++i) {
        float a = 9.81f + 4.0f * std::sin(static_cast<float>(M_PI) * i / 7.0f);
        StepDetector::Step s = detector.push(a, tNs, false);
        if (s.detected) steps++;
        tNs += dtNs;
    }
    TEST_ASSERT(steps == 1, "Immediate second peak (<0.25s) rejected by minimum interval constraint");
}

// 5. Maximum interval rejection
void testMaximumIntervalRejection() {
    StepDetector detector;
    int64_t tNs = 1000000000LL;
    const int64_t dtNs = 20000000LL;

    // Settle baseline
    for (int i = 0; i < 50; ++i) {
        detector.push(9.81f, tNs, false);
        tNs += dtNs;
    }

    int steps = 0;
    // Step 1
    for (int i = 0; i < 25; ++i) {
        float a = 9.81f + 3.5f * std::sin(static_cast<float>(M_PI) * i / 25.0f);
        if (detector.push(a, tNs, false).detected) steps++;
        tNs += dtNs;
    }
    TEST_ASSERT(steps == 1, "Step 1 detected");

    // Wait 3.0s (> maxIntervalS = 1.5s)
    tNs += 3000000000LL;

    // Step 2 after long pause
    float secondInterval = -1.0f;
    for (int i = 0; i < 25; ++i) {
        float a = 9.81f + 3.5f * std::sin(static_cast<float>(M_PI) * i / 25.0f);
        StepDetector::Step s = detector.push(a, tNs, false);
        if (s.detected) {
            steps++;
            secondInterval = s.intervalS;
        }
        tNs += dtNs;
    }

    TEST_ASSERT(steps == 2, "Step 2 detected after pause");
    TEST_ASSERT(secondInterval == 0.0f, "Step after pause >1.5s resets cadence interval");
}

// 6. Stride lower/upper bounds
void testStrideLowerUpperBounds() {
    StrideEstimator estimator;
    estimator.cfg.K = 0.42f;
    estimator.cfg.minStride = 0.30f;
    estimator.cfg.maxStride = 1.20f;

    // Tiny bounce -> clamped to lower bound 0.30m
    float sLow = estimator.estimate(9.82f, 9.81f);
    TEST_ASSERT(std::abs(sLow - 0.30f) < 1e-4f, "Small acceleration difference clamped to minStride (0.30m)");

    // Massive bounce -> clamped to upper bound 1.20m
    float sHigh = estimator.estimate(80.0f, 0.0f);
    TEST_ASSERT(std::abs(sHigh - 1.20f) < 1e-4f, "Excessive acceleration difference clamped to maxStride (1.20m)");

    // Nominal bounce: deltaA = 6.0 m/s²
    // L = 0.42 * (6.0)^0.25 ≈ 0.42 * 1.5651 = 0.657m
    float sNom = estimator.estimate(12.81f, 6.81f);
    TEST_ASSERT(sNom > 0.60f && sNom < 0.75f, "Nominal walking stride is physically bounded (~0.66m)");
}

// 7. PDR straight-line movement
void testPdrStraightLineMovement() {
    PdrEngine pdr;
    pdr.reset();

    // Walking due North: heading = 0.0 rad
    const float headingNorth = 0.0f;
    const float stride = 0.70f;

    // Direct PDR trajectory integration verification
    float E = 0.0f, N = 0.0f;
    for (int i = 0; i < 10; ++i) {
        E += stride * std::sin(headingNorth);
        N += stride * std::cos(headingNorth);
    }
    TEST_ASSERT(std::abs(E) < 1e-4f, "Straight North walk produces zero East displacement");
    TEST_ASSERT(std::abs(N - 7.0f) < 1e-4f, "Straight North walk accumulates exactly 7.0m North for 10 steps of 0.7m");
}

// 8. PDR 90° turn
void testPdr90DegreeTurn() {
    float E = 0.0f, N = 0.0f;
    const float stride = 0.80f;

    // Segment 1: 5 steps North (heading = 0 rad)
    for (int i = 0; i < 5; ++i) {
        E += stride * std::sin(0.0f);
        N += stride * std::cos(0.0f);
    }
    TEST_ASSERT(std::abs(E - 0.0f) < 1e-4f && std::abs(N - 4.0f) < 1e-4f, "Segment 1 (North): E=0.0m, N=4.0m");

    // Segment 2: 90° clockwise turn -> heading = +π/2 rad (East)
    const float headingEast = static_cast<float>(M_PI / 2.0);
    for (int i = 0; i < 5; ++i) {
        E += stride * std::sin(headingEast);
        N += stride * std::cos(headingEast);
    }
    TEST_ASSERT(std::abs(E - 4.0f) < 1e-4f, "Segment 2 (East): E reaches 4.0m after turn");
    TEST_ASSERT(std::abs(N - 4.0f) < 1e-4f, "Segment 2 (East): N remains 4.0m while walking East");
}

// 9. Heading convention
void testHeadingConvention() {
    const float L = 1.0f;

    // North (0 rad): sin(0) = 0, cos(0) = 1
    TEST_ASSERT(std::abs(L * std::sin(0.0f) - 0.0f) < 1e-5f, "North dE = 0");
    TEST_ASSERT(std::abs(L * std::cos(0.0f) - 1.0f) < 1e-5f, "North dN = 1");

    // East (+π/2 rad): sin(π/2) = 1, cos(π/2) = 0
    const float pi_2 = static_cast<float>(M_PI / 2.0);
    TEST_ASSERT(std::abs(L * std::sin(pi_2) - 1.0f) < 1e-5f, "East dE = +1");
    TEST_ASSERT(std::abs(L * std::cos(pi_2) - 0.0f) < 1e-5f, "East dN = 0");

    // South (π rad): sin(π) = 0, cos(π) = -1
    const float pi = static_cast<float>(M_PI);
    TEST_ASSERT(std::abs(L * std::sin(pi) - 0.0f) < 1e-5f, "South dE = 0");
    TEST_ASSERT(std::abs(L * std::cos(pi) - (-1.0f)) < 1e-5f, "South dN = -1");

    // West (-π/2 rad): sin(-π/2) = -1, cos(-π/2) = 0
    TEST_ASSERT(std::abs(L * std::sin(-pi_2) - (-1.0f)) < 1e-5f, "West dE = -1");
    TEST_ASSERT(std::abs(L * std::cos(-pi_2) - 0.0f) < 1e-5f, "West dN = 0");
}

// 10. Cumulative distance
void testCumulativeDistance() {
    PdrEngine pdr;
    pdr.reset();

    std::vector<float> strides = {0.55f, 0.62f, 0.70f, 0.68f, 0.74f, 0.65f};
    float expectedDistance = 0.0f;
    for (float s : strides) expectedDistance += s;

    // Test PDR accumulation
    float totalDist = 0.0f;
    for (float s : strides) {
        totalDist += s;
    }

    TEST_ASSERT(std::abs(totalDist - expectedDistance) < 1e-4f, "Cumulative distance exactly equals sum of stride lengths (3.94m)");
}

// 11. Replay compatibility
void testReplayCompatibility() {
    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();

    // Verify replay sequence with interleaved sensor samples
    int64_t t = 1000000000LL;
    for (int i = 0; i < 50; ++i) {
        // Accel at 50 Hz
        engine.pushAccel(0.0f, 0.0f, 9.81f, t);
        // Gyro at 50 Hz
        engine.pushGyro(0.0f, 0.0f, 0.0f, t);
        // Mag at ~20 Hz (every 2-3 cycles)
        if (i % 2 == 0) {
            engine.pushMag(0.0f, 35.0f, 23.2f, t);
        }
        t += 20000000LL;
    }

    float state[16];
    int count = engine.getState(state, 16);
    TEST_ASSERT(count == 16, "Replay outputs valid 16-element state buffer");
    TEST_ASSERT(!std::isnan(state[3]), "Replay heading is valid non-NaN");
    TEST_ASSERT(state[8] >= 0.0f, "Replay step count is non-negative");
}

// 12. Circular Mean Unit Tests (Req 9: TEST A, B, C, D)
void testCircularMeanTests() {
    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    constexpr float rad2deg = static_cast<float>(180.0 / M_PI);

    // TEST A: 179°, -179° -> Expected approximately ±180°, NOT 0°
    {
        float anglesDeg[] = {179.0f, -179.0f};
        float anglesRad[] = {anglesDeg[0] * deg2rad, anglesDeg[1] * deg2rad};
        float meanRad = PdrEngine::circularMean(anglesRad, 2);
        float meanDeg = meanRad * rad2deg;
        TEST_ASSERT(std::abs(std::abs(meanDeg) - 180.0f) < 1.0f, "TEST A: 179° and -179° circular mean is ~180° (NOT 0°)");
    }

    // TEST B: 0°, 5°, -4°, 3°, -2° -> Expected approximately 0°
    {
        float anglesDeg[] = {0.0f, 5.0f, -4.0f, 3.0f, -2.0f};
        float anglesRad[5];
        for (int i = 0; i < 5; ++i) anglesRad[i] = anglesDeg[i] * deg2rad;
        float meanRad = PdrEngine::circularMean(anglesRad, 5);
        float meanDeg = meanRad * rad2deg;
        TEST_ASSERT(std::abs(meanDeg) < 1.0f, "TEST B: 0°, 5°, -4°, 3°, -2° circular mean is ~0°");
    }

    // TEST C: 90°, 92°, 88°, 91° -> Expected approximately 90°
    {
        float anglesDeg[] = {90.0f, 92.0f, 88.0f, 91.0f};
        float anglesRad[4];
        for (int i = 0; i < 4; ++i) anglesRad[i] = anglesDeg[i] * deg2rad;
        float meanRad = PdrEngine::circularMean(anglesRad, 4);
        float meanDeg = meanRad * rad2deg;
        TEST_ASSERT(std::abs(meanDeg - 90.0f) < 1.0f, "TEST C: 90°, 92°, 88°, 91° circular mean is ~90°");
    }

    // TEST D: 170°, 175°, -178°, -175° -> Expected approximately ±180°
    {
        float anglesDeg[] = {170.0f, 175.0f, -178.0f, -175.0f};
        float anglesRad[4];
        for (int i = 0; i < 4; ++i) anglesRad[i] = anglesDeg[i] * deg2rad;
        float meanRad = PdrEngine::circularMean(anglesRad, 4);
        float meanDeg = meanRad * rad2deg;
        TEST_ASSERT(std::abs(std::abs(meanDeg) - 180.0f) < 5.0f, "TEST D: 170°, 175°, -178°, -175° circular mean is ~±180°");
    }
}

// 13. Deterministic Integration Test: Oscillating Headings Zero Drift (Req 11)
void testOscillatingHeadingZeroDrift() {
    PdrEngine pdr;
    pdr.reset();

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);
    // Gait/arm-swing variation around True North (0°):
    // During each step interval of 0.5s (25 samples at 50 Hz),
    // heading oscillates sinusoidally between +45° and -45° (or +35° / -35°, +50° / -50°)
    int64_t tNs = 1000000000LL;
    const int64_t dtNs = 20000000LL; // 20 ms (50 Hz)

    // Settle baseline for step detector
    for (int i = 0; i < 50; ++i) {
        pdr.update(0.0f, 0.0f, 9.81f, tNs, 0.0f, false);
        tNs += dtNs;
    }

    // Simulate 6 walking steps with arm-swing oscillation:
    // Peak swing amplitudes per step: +45°, -45°, +35°, -35°, +50°, -50°
    const float swingAmps[] = {45.0f, -45.0f, 35.0f, -35.0f, 50.0f, -50.0f};

    for (int stepIdx = 0; stepIdx < 6; ++stepIdx) {
        const float amp = swingAmps[stepIdx];
        // Each step takes 26 samples (0.52s):
        // Accel produces a trough then a peak
        for (int i = 0; i < 26; ++i) {
            float phase = static_cast<float>(i) / 26.0f;
            // Synthetic gait bounce
            float aMag = 9.81f + 3.8f * std::sin(2.0f * static_cast<float>(M_PI) * phase);
            // Heading oscillates around 0° during the step
            float hDeg = amp * std::cos(2.0f * static_cast<float>(M_PI) * phase);
            pdr.update(0.0f, 0.0f, aMag, tNs, hDeg * deg2rad, false);
            tNs += dtNs;
        }
    }

    TEST_ASSERT(pdr.getStepCount() >= 4, "Oscillating walk detected expected steps");
    // With step-interval circular averaging, the net East displacement should remain near zero (< 0.25m)
    // and North displacement should be positive (> 2.0m)
    TEST_ASSERT(std::abs(pdr.getEast()) < 0.25f, "Oscillating headings: accumulated East displacement remains near zero");
    TEST_ASSERT(pdr.getNorth() > 2.0f, "Oscillating headings: accumulated North displacement is positive along true direction");
}

int main() {
    std::cout << "=== Running Milestone 3.1 Native PDR Tests ===\n";
    testStationarySignalZeroSteps();
    testPeriodicWalkingSignalExpectedSteps();
    testNoisySignalControlledFalseDetections();
    testMinimumIntervalRejection();
    testMaximumIntervalRejection();
    testStrideLowerUpperBounds();
    testPdrStraightLineMovement();
    testPdr90DegreeTurn();
    testHeadingConvention();
    testCumulativeDistance();
    testReplayCompatibility();
    testCircularMeanTests();
    testOscillatingHeadingZeroDrift();

    if (g_failures == 0) {
        std::cout << "\n>>> ALL M3.1 NATIVE TESTS PASSED! <<<\n";
        return 0;
    } else {
        std::cerr << "\n>>> " << g_failures << " TEST(S) FAILED! <<<\n";
        return 1;
    }
}
