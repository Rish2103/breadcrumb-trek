#include "../../native/ahrs/MathTypes.hpp"
#include "../../native/ahrs/Madgwick9DOF.hpp"
#include "../../native/ahrs/MagneticReliability.hpp"
#include "../../native/ahrs/StationaryDetector.hpp"
#include "../../native/ahrs/NavigationEngine.hpp"
#include "../../native/pdr/StepDetector.hpp"
#include "../../native/pdr/StrideEstimator.hpp"
#include "../../native/pdr/PdrEngine.hpp"
#include "../../native/trail/TrailPoint.hpp"
#include "../../native/trail/TrailMemory.hpp"
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

// 1. Empty trail
void testEmptyTrail() {
    TrailMemory trail;
    TEST_ASSERT(trail.getPointCount() == 0, "Empty trail has 0 points");
    TEST_ASSERT(trail.getState() == TrailRecordingState::IDLE, "Initial trail state is IDLE");
    TEST_ASSERT(trail.getTotalDistance() == 0.0f, "Empty trail distance is 0.0m");
    TEST_ASSERT(trail.getStartPoint() == nullptr, "Empty trail start point is null");
    TEST_ASSERT(trail.getLatestPoint() == nullptr, "Empty trail latest point is null");
}

// 2. Single point
void testSinglePoint() {
    TrailMemory trail;
    int64_t t0 = 1000000000LL;
    trail.startRecording(t0, 0.45f);

    TEST_ASSERT(trail.getState() == TrailRecordingState::RECORDING, "Trail state is RECORDING");
    TEST_ASSERT(trail.getPointCount() == 1, "Start recording establishes exactly 1 anchor point (P0)");
    const TrailPoint* p0 = trail.getStartPoint();
    TEST_ASSERT(p0 != nullptr, "Start point exists");
    TEST_ASSERT(p0->sequenceIndex == 0, "Anchor point sequence index is 0");
    TEST_ASSERT(p0->east == 0.0f && p0->north == 0.0f, "Anchor point P0 is at local origin (0, 0)");
    TEST_ASSERT(p0->stepCount == 0, "Anchor point step count is 0");
    TEST_ASSERT(p0->cumulativeDistance == 0.0f, "Anchor point distance is 0.0m");
}

// 3. Ordered append
void testOrderedAppend() {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f);

    for (int i = 1; i <= 5; ++i) {
        t += 500000000LL; // +0.5s
        bool ok = trail.addPoint(t, 0.0f, i * 0.70f, 0.0f, i, i * 0.70f);
        TEST_ASSERT(ok, "Ordered point append succeeds");
    }

    TEST_ASSERT(trail.getPointCount() == 6, "Trail contains 6 ordered points (P0..P5)");
    TEST_ASSERT(std::abs(trail.getTotalDistance() - 3.50f) < 1e-4f, "Trail distance matches cumulative total (3.50m)");

    for (size_t i = 0; i < trail.getPointCount(); ++i) {
        const TrailPoint* p = trail.getPoint(i);
        TEST_ASSERT(p->sequenceIndex == static_cast<uint32_t>(i), "Point sequence index strictly matches array index");
    }
}

// 4. Timestamp ordering
void testTimestampOrdering() {
    TrailMemory trail;
    int64_t t = 2000000000LL;
    trail.startRecording(t, 0.0f);

    // Forward timestamp accepted
    bool ok1 = trail.addPoint(t + 500000000LL, 0.0f, 1.0f, 0.0f, 1, 1.0f);
    TEST_ASSERT(ok1, "Monotonically increasing timestamp accepted");

    // Retrograde timestamp rejected
    bool okRetrograde = trail.addPoint(t + 400000000LL, 0.0f, 2.0f, 0.0f, 2, 2.0f);
    TEST_ASSERT(!okRetrograde, "Retrograde timestamp rejected");

    // Duplicate timestamp rejected
    bool okDuplicate = trail.addPoint(t + 500000000LL, 0.0f, 2.0f, 0.0f, 2, 2.0f);
    TEST_ASSERT(!okDuplicate, "Duplicate timestamp rejected");

    TEST_ASSERT(trail.getPointCount() == 2, "Only valid monotonic points appended");
}

// 5. Sequence ordering
void testSequenceOrdering() {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f);

    for (int i = 1; i <= 10; ++i) {
        t += 300000000LL;
        trail.addPoint(t, static_cast<float>(i), static_cast<float>(i), 0.0f, i, i * 1.414f);
    }

    TEST_ASSERT(trail.getPointCount() == 11, "11 points total in sequence");
    bool strictlyMonotonic = true;
    for (size_t i = 1; i < trail.getPointCount(); ++i) {
        if (trail.getPoint(i)->sequenceIndex != trail.getPoint(i - 1)->sequenceIndex + 1) {
            strictlyMonotonic = false;
        }
    }
    TEST_ASSERT(strictlyMonotonic, "Sequence indices are strictly monotonic (i = i_prev + 1)");
}

// 6. Invalid point rejection
void testInvalidPointRejection() {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f);

    // NaN East
    bool okNanE = trail.addPoint(t + 100000000LL, NAN, 1.0f, 0.0f, 1, 1.0f);
    TEST_ASSERT(!okNanE, "NaN East coordinate rejected");

    // Infinite North
    bool okInfN = trail.addPoint(t + 200000000LL, 1.0f, INFINITY, 0.0f, 1, 1.0f);
    TEST_ASSERT(!okInfN, "Infinite North coordinate rejected");

    // NaN heading
    bool okNanHeading = trail.addPoint(t + 300000000LL, 1.0f, 1.0f, NAN, 1, 1.0f);
    TEST_ASSERT(!okNanHeading, "NaN heading rejected");

    // Non-positive timestamp
    bool okZeroT = trail.addPoint(0, 1.0f, 1.0f, 0.0f, 1, 1.0f);
    TEST_ASSERT(!okZeroT, "Zero timestamp rejected");

    TEST_ASSERT(trail.getPointCount() == 1, "All invalid/corrupted points rejected without polluting trail");
}

// 7. Reset
void testReset() {
    TrailMemory trail;
    trail.startRecording(1000000000LL, 0.0f);
    trail.addPoint(1500000000LL, 1.0f, 1.0f, 0.0f, 1, 1.41f);
    TEST_ASSERT(trail.getPointCount() == 2, "Trail has 2 points before reset");

    trail.reset();
    TEST_ASSERT(trail.getPointCount() == 0, "Reset completely clears stored points");
    TEST_ASSERT(trail.getState() == TrailRecordingState::IDLE, "Reset restores IDLE state");
    TEST_ASSERT(trail.getStartPoint() == nullptr, "Start point is null after reset");
    TEST_ASSERT(trail.getLatestPoint() == nullptr, "Latest point is null after reset");
}

// 8. Journey finalization
void testJourneyFinalization() {
    TrailMemory trail;
    trail.startRecording(1000000000LL, 0.0f);
    trail.addPoint(1500000000LL, 0.0f, 1.0f, 0.0f, 1, 1.0f);

    trail.stopRecording(2000000000LL);
    TEST_ASSERT(trail.getState() == TrailRecordingState::RETURN_READY, "Stop recording transitions state to RETURN_READY");

    // Attempting to append while frozen in RETURN_READY must fail
    bool okAppendAfterStop = trail.addPoint(2500000000LL, 0.0f, 2.0f, 0.0f, 2, 2.0f);
    TEST_ASSERT(!okAppendAfterStop, "Appending rejected when trail is frozen in RETURN_READY");
    TEST_ASSERT(trail.getPointCount() == 2, "Trail point count remains frozen");
}

// 9. Straight trajectory
void testStraightTrajectory() {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f); // Facing North

    for (int i = 1; i <= 10; ++i) {
        t += 500000000LL;
        trail.addPoint(t, 0.0f, i * 0.75f, 0.0f, i, i * 0.75f);
    }

    TEST_ASSERT(trail.getPointCount() == 11, "Straight trajectory has 11 points (P0..P10)");
    const TrailPoint* latest = trail.getLatestPoint();
    TEST_ASSERT(latest != nullptr, "Latest point exists");
    TEST_ASSERT(std::abs(latest->east - 0.0f) < 1e-4f, "East coordinate remains 0.0m along straight North walk");
    TEST_ASSERT(std::abs(latest->north - 7.50f) < 1e-4f, "North coordinate reaches 7.50m");
    TEST_ASSERT(std::abs(trail.getTotalDistance() - 7.50f) < 1e-4f, "Total trail distance is 7.50m");
}

// 10. 90-degree turn
void test90DegreeTurn() {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f);

    // Segment 1: 5 steps North
    for (int i = 1; i <= 5; ++i) {
        t += 500000000LL;
        trail.addPoint(t, 0.0f, i * 0.80f, 0.0f, i, i * 0.80f);
    }
    TEST_ASSERT(std::abs(trail.getLatestPoint()->north - 4.0f) < 1e-4f, "Segment 1 ends at N=4.0m, E=0.0m");

    // Segment 2: 5 steps East
    const float headingEast = static_cast<float>(M_PI / 2.0);
    for (int i = 1; i <= 5; ++i) {
        t += 500000000LL;
        trail.addPoint(t, i * 0.80f, 4.0f, headingEast, 5 + i, 4.0f + i * 0.80f);
    }

    TEST_ASSERT(trail.getPointCount() == 11, "11 points total in L-shaped route");
    const TrailPoint* corner = trail.getPoint(5);
    const TrailPoint* destination = trail.getLatestPoint();

    TEST_ASSERT(std::abs(corner->east - 0.0f) < 1e-4f && std::abs(corner->north - 4.0f) < 1e-4f, "Corner breadcrumb P5 is exactly at (0.0, 4.0)");
    TEST_ASSERT(std::abs(destination->east - 4.0f) < 1e-4f && std::abs(destination->north - 4.0f) < 1e-4f, "Destination breadcrumb P10 is at (4.0, 4.0)");
    TEST_ASSERT(std::abs(trail.getTotalDistance() - 8.00f) < 1e-4f, "Total L-shaped distance is 8.00m");
}

// 11. Loop/self-crossing preservation
void testLoopSelfCrossingPreservation() {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f); // P0 at (0, 0)

    // Square loop: (0,0) -> (0,5) -> (5,5) -> (5,0) -> (0,0)
    t += 500000000LL;
    trail.addPoint(t, 0.0f, 5.0f, 0.0f, 1, 5.0f); // P1
    t += 500000000LL;
    trail.addPoint(t, 5.0f, 5.0f, 1.57f, 2, 10.0f); // P2
    t += 500000000LL;
    trail.addPoint(t, 5.0f, 0.0f, 3.14f, 3, 15.0f); // P3
    t += 500000000LL;
    trail.addPoint(t, 0.0f, 0.0f, -1.57f, 4, 20.0f); // P4 (crosses back to origin!)

    TEST_ASSERT(trail.getPointCount() == 5, "Loop contains 5 distinct breadcrumbs");

    const TrailPoint* p0 = trail.getPoint(0);
    const TrailPoint* p4 = trail.getPoint(4);

    TEST_ASSERT(p0->sequenceIndex == 0, "P0 sequenceIndex is 0");
    TEST_ASSERT(p4->sequenceIndex == 4, "P4 sequenceIndex is 4");
    TEST_ASSERT(std::abs(p0->cumulativeDistance - 0.0f) < 1e-4f, "P0 distance is 0.0m");
    TEST_ASSERT(std::abs(p4->cumulativeDistance - 20.0f) < 1e-4f, "P4 distance is 20.0m");

    // Both are at (0, 0) spatially, but temporally distinct in sequence
    TEST_ASSERT(std::abs(p0->east - p4->east) < 1e-4f && std::abs(p0->north - p4->north) < 1e-4f, "P0 and P4 share spatial coordinates (0, 0)");
    TEST_ASSERT(p0->timestampNs < p4->timestampNs, "P0 and P4 retain distinct chronological timestamps");
}

// 12. Replay reproducibility
void testReplayReproducibility() {
    NavigationEngine& engine = NavigationEngine::instance();

    // Run 1
    engine.reset();
    engine.startTrailRecording();

    int64_t t = 1000000000LL;
    const int64_t dt = 20000000LL;

    // Simulate 3 steps
    for (int step = 0; step < 3; ++step) {
        for (int s = 0; s < 25; ++s) {
            float phase = 2.0f * static_cast<float>(M_PI) * s / 25.0f;
            float az = 9.81f + 4.0f * std::sin(phase);
            engine.pushAccel(0.0f, 0.0f, az, t);
            engine.pushGyro(0.0f, 0.0f, 0.0f, t);
            t += dt;
        }
    }
    engine.stopTrailRecording();

    size_t count1 = engine.getTrailMemory().getPointCount();
    float dist1 = engine.getTrailMemory().getTotalDistance();
    float latestE1 = engine.getTrailMemory().getLatestPoint()->east;
    float latestN1 = engine.getTrailMemory().getLatestPoint()->north;

    // Run 2: Exact same replay sequence from clean reset
    engine.reset();
    engine.startTrailRecording();

    t = 1000000000LL;
    for (int step = 0; step < 3; ++step) {
        for (int s = 0; s < 25; ++s) {
            float phase = 2.0f * static_cast<float>(M_PI) * s / 25.0f;
            float az = 9.81f + 4.0f * std::sin(phase);
            engine.pushAccel(0.0f, 0.0f, az, t);
            engine.pushGyro(0.0f, 0.0f, 0.0f, t);
            t += dt;
        }
    }
    engine.stopTrailRecording();

    size_t count2 = engine.getTrailMemory().getPointCount();
    float dist2 = engine.getTrailMemory().getTotalDistance();
    float latestE2 = engine.getTrailMemory().getLatestPoint()->east;
    float latestN2 = engine.getTrailMemory().getLatestPoint()->north;

    TEST_ASSERT(count1 == count2, "Replay point count is 100% reproducible");
    TEST_ASSERT(std::abs(dist1 - dist2) < 1e-5f, "Replay distance is 100% reproducible");
    TEST_ASSERT(std::abs(latestE1 - latestE2) < 1e-5f && std::abs(latestN1 - latestN2) < 1e-5f, "Replay trajectory coordinates are 100% reproducible");
}

int main() {
    std::cout << "=== Running Milestone 4 Native Trail Memory Tests ===\n";
    testEmptyTrail();
    testSinglePoint();
    testOrderedAppend();
    testTimestampOrdering();
    testSequenceOrdering();
    testInvalidPointRejection();
    testReset();
    testJourneyFinalization();
    testStraightTrajectory();
    test90DegreeTurn();
    testLoopSelfCrossingPreservation();
    testReplayReproducibility();

    if (g_failures == 0) {
        std::cout << "\n>>> ALL M4 NATIVE TESTS PASSED! <<<\n";
        return 0;
    } else {
        std::cerr << "\n>>> " << g_failures << " TEST(S) FAILED! <<<\n";
        return 1;
    }
}
