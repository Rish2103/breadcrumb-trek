#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <string>

#include "ahrs/NavigationEngine.hpp"
#include "guidance/ReverseNavigator.hpp"
#include "trail/TrailMemory.hpp"

using namespace breadcrumb;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")" << std::endl; \
            exit(1); \
        } else { \
            std::cout << "  [PASS] " << msg << std::endl; \
        } \
    } while (0)

// Helper: build synthetic trail
TrailMemory buildTrail(const std::vector<P2>& waypoints, float stepLen = 0.8f, float cMag = 1.0f) {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f); // P0 at (0,0)

    uint32_t step = 0;
    float cumDist = 0.0f;
    P2 prev = {0.0, 0.0};

    for (size_t i = 1; i < waypoints.size(); ++i) {
        const P2 curr = waypoints[i];
        const double dE = curr.e - prev.e;
        const double dN = curr.n - prev.n;
        const double dist = std::sqrt(dE * dE + dN * dN);
        const float segHeading = static_cast<float>(std::atan2(dE, dN));

        int nSteps = std::max(1, static_cast<int>(std::round(dist / stepLen)));
        for (int s = 1; s <= nSteps; ++s) {
            t += 400000000LL;
            step++;
            double frac = static_cast<double>(s) / nSteps;
            float e = static_cast<float>(prev.e + dE * frac);
            float n = static_cast<float>(prev.n + dN * frac);
            cumDist += static_cast<float>(dist / nSteps);
            trail.addPoint(t, e, n, segHeading, step, cumDist, cMag);
        }
        prev = curr;
    }
    trail.stopRecording(t + 100000000LL);
    return trail;
}

// 1. testReverseOrdering: Verify Pn -> P(n-1) -> ... -> P0
void testReverseOrdering() {
    std::cout << "\nTest 1: Reverse Ordering" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 5}, {5, 5}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    bool ok = nav.startReturn(trail);
    TEST_ASSERT(ok, "Reverse navigation initialized on trail");
    TEST_ASSERT(nav.getPointCount() == trail.getPointCount(), "Point count matches");

    const size_t N = nav.getPointCount();
    TrailPoint firstRev, lastRev;
    nav.getReversePoint(0, firstRev);
    nav.getReversePoint(N - 1, lastRev);

    TEST_ASSERT(firstRev.sequenceIndex == N - 1, "Reverse index 0 maps to Pn (sequence index N-1)");
    TEST_ASSERT(lastRev.sequenceIndex == 0, "Reverse index N-1 maps to P0 (sequence index 0)");
    TEST_ASSERT(std::abs(firstRev.east - 5.0f) < 1e-3f && std::abs(firstRev.north - 5.0f) < 1e-3f, "Pn is at (5.0, 5.0)");
    TEST_ASSERT(std::abs(lastRev.east - 0.0f) < 1e-3f && std::abs(lastRev.north - 0.0f) < 1e-3f, "P0 is at (0.0, 0.0)");
}

// 2. testReversePointAccessor: Verify reverse accessor returns correct original TrailPoint metadata
void testReversePointAccessor() {
    std::cout << "\nTest 2: Reverse Point Accessor" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 10}};
    TrailMemory trail = buildTrail(wps, 1.0f, 0.85f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    const size_t N = nav.getPointCount();
    for (size_t r = 0; r < N; ++r) {
        TrailPoint pt;
        bool found = nav.getReversePoint(r, pt);
        TEST_ASSERT(found, "Point retrieved successfully");
        const TrailPoint* orig = trail.getPoint(N - 1 - r);
        TEST_ASSERT(pt.sequenceIndex == orig->sequenceIndex, "sequenceIndex preserved");
        TEST_ASSERT(pt.timestampNs == orig->timestampNs, "timestampNs preserved");
        TEST_ASSERT(pt.east == orig->east, "east coordinate preserved");
        TEST_ASSERT(pt.north == orig->north, "north coordinate preserved");
        TEST_ASSERT(pt.cMag == orig->cMag, "cMag preserved");
        TEST_ASSERT(pt.cumulativeDistance == orig->cumulativeDistance, "cumulativeDistance preserved");
    }
}

// 3. testReverseHeading: Verify reverse heading points toward P0
void testReverseHeading() {
    std::cout << "\nTest 3: Reverse Heading" << std::endl;
    // Walk North: Forward heading is 0.0 rad (North). Reverse heading must be +/-pi rad (South).
    P2 a{0.0, 10.0};
    P2 b{0.0, 0.0};
    double revBearing = ReverseNavigator::computeSegmentBearing(a, b);
    TEST_ASSERT(std::abs(std::abs(revBearing) - M_PI) < 1e-5, "Reverse heading for North walk points South (pi rad)");

    // Walk East: Forward heading is pi/2. Reverse heading must be -pi/2 (West).
    P2 c{10.0, 0.0};
    P2 d{0.0, 0.0};
    double revBearingEast = ReverseNavigator::computeSegmentBearing(c, d);
    TEST_ASSERT(std::abs(revBearingEast - (-M_PI / 2.0)) < 1e-5, "Reverse heading for East walk points West (-pi/2 rad)");
}

// 4. testSegmentProjection: Verify point-to-segment projection
void testSegmentProjection() {
    std::cout << "\nTest 4: Segment Projection" << std::endl;
    P2 a{0.0, 0.0};
    P2 b{10.0, 0.0};

    // User at (5.0, 2.0): projection Q should be (5.0, 0.0), dist 2.0
    P2 u1{5.0, 2.0};
    P2 q; double u, dist;
    ReverseNavigator::projectOnSegment(u1, a, b, q, u, dist);
    TEST_ASSERT(std::abs(q.e - 5.0) < 1e-5 && std::abs(q.n - 0.0) < 1e-5, "Projection Q is (5.0, 0.0)");
    TEST_ASSERT(std::abs(u - 0.5) < 1e-5, "Projection parameter u is 0.5");
    TEST_ASSERT(std::abs(dist - 2.0) < 1e-5, "Distance is 2.0m");

    // Clamping before start: user at (-3.0, 0.0) -> Q should be A (0.0, 0.0)
    P2 u2{-3.0, 0.0};
    ReverseNavigator::projectOnSegment(u2, a, b, q, u, dist);
    TEST_ASSERT(std::abs(q.e - 0.0) < 1e-5 && std::abs(q.n - 0.0) < 1e-5, "Clamped to endpoint A");
    TEST_ASSERT(std::abs(u - 0.0) < 1e-5, "u clamped to 0.0");
    TEST_ASSERT(std::abs(dist - 3.0) < 1e-5, "Distance is 3.0m");

    // Clamping after end: user at (14.0, 0.0) -> Q should be B (10.0, 0.0)
    P2 u3{14.0, 0.0};
    ReverseNavigator::projectOnSegment(u3, a, b, q, u, dist);
    TEST_ASSERT(std::abs(q.e - 10.0) < 1e-5 && std::abs(q.n - 0.0) < 1e-5, "Clamped to endpoint B");
    TEST_ASSERT(std::abs(u - 1.0) < 1e-5, "u clamped to 1.0");
    TEST_ASSERT(std::abs(dist - 4.0) < 1e-5, "Distance is 4.0m");
}

// 5. testZeroLengthSegment: Verify no divide-by-zero or crash
void testZeroLengthSegment() {
    std::cout << "\nTest 5: Zero-Length Segment Handling" << std::endl;
    P2 a{5.0, 5.0};
    P2 b{5.0, 5.0};
    P2 p{5.0, 8.0};

    P2 q; double u, dist;
    ReverseNavigator::projectOnSegment(p, a, b, q, u, dist);
    TEST_ASSERT(std::abs(q.e - 5.0) < 1e-5 && std::abs(q.n - 5.0) < 1e-5, "Zero-length segment defaults safely to point");
    TEST_ASSERT(std::abs(dist - 3.0) < 1e-5, "Distance correctly computed to point");
    TEST_ASSERT(!std::isnan(dist) && !std::isinf(dist), "No NaN or Infinity produced");
}

// 6. testStraightTrailMatching: Verify user following straight trail matches expected segment
void testStraightTrailMatching() {
    std::cout << "\nTest 6: Straight Trail Matching" << std::endl;
    // Forward: (0,0) -> (0, 20) with 20 steps (1m each)
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // User at (0.0, 19.0) heading South (-pi): should match segment 0 (P20 -> P19)
    GuidanceOut g = nav.update({0.0, 19.0}, -M_PI, 1.0);
    TEST_ASSERT(g.segIndex == 0 || g.segIndex == 1, "Matches initial reverse segment near Pn");
    TEST_ASSERT(g.state == NavigationState::ON_TRAIL, "Navigation state is ON_TRAIL");
    TEST_ASSERT(std::abs(g.xte) < 0.1, "Cross-track error is near zero along trail center");
}

// 7. testProgressMonotonicity: Verify reverse progress advances toward P0 without uncontrolled oscillation
void testProgressMonotonicity() {
    std::cout << "\nTest 7: Progress Monotonicity" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 10}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    int prevProg = 0;
    // Walk back step-by-step from 10m to 1m
    for (double n = 9.5; n >= 1.0; n -= 0.5) {
        GuidanceOut g = nav.update({0.0, n}, -M_PI, 1.0);
        TEST_ASSERT(g.reverseProgressIndex >= prevProg, "Progress index moves monotonically forward along reverse path");
        prevProg = g.reverseProgressIndex;
    }
}

// 8. testProgressWindow: Verify candidates outside allowed search window are not selected
void testProgressWindow() {
    std::cout << "\nTest 8: Progress Search Window Bounding" << std::endl;
    // 30 segment trail
    std::vector<P2> wps = {{0, 0}, {0, 30}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    ReverseNavigator::Cfg cfg = nav.getConfig();
    cfg.backWin = 1;
    cfg.fwdWin = 3;
    nav.setConfig(cfg);
    nav.startReturn(trail);

    // Initial segment is 0. User at (0, 15) is 15 segments away (outside forward window 3).
    // Gating and window must prevent jumping immediately to segment 15!
    GuidanceOut g = nav.update({0.0, 15.0}, -M_PI, 1.0);
    TEST_ASSERT(g.segIndex <= cfg.fwdWin, "Matched segment is constrained within forward search window");
}

// 9. testCrossTrackError: Explicit verification of XTE sign convention
void testCrossTrackError() {
    std::cout << "\nTest 9: Cross-Track Error & Sign Convention" << std::endl;
    // Reverse segment traveling NORTH: A = (0, -10), B = (0, 0).
    // Forward walk: from (0, 0) South to (0, -10).
    // Reverse traversal: from (0, -10) North to (0, 0).
    // Tangent t = (0, 1) (North). Left normal n_left = [-tN, tE] = [-1, 0] (West).
    // User placed on EAST side: U = (+2.0, -5.0).
    // Projected point on trail Q = (0.0, -5.0).
    // Vector from user to trail: e = Q - U = (-2.0, 0.0) (points West).
    // e_perp = e . n_left = (-2.0)*(-1.0) + 0 = +2.0m.
    // Trail is to the user's LEFT -> xte must be POSITIVE (+).

    std::vector<P2> wps = {{0, 0}, {0, -10}}; // Forward: South (0,0)->(0,-10). Reverse: North (0,-10)->(0,0)
    TrailMemory trail = buildTrail(wps, 2.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // East side test: U = (+2.0, -5.0), heading North (0.0)
    GuidanceOut gEast = nav.update({2.0, -5.0}, 0.0, 1.0);
    std::cout << "  User on East of Northbound trail: XTE = " << gEast.xte << " m" << std::endl;
    TEST_ASSERT(gEast.xte > 1.9 && gEast.xte < 2.1, "User on East side: XTE is POSITIVE (+2.0m), trail is to user's LEFT");
    TEST_ASSERT(std::abs(gEast.absXte - 2.0) < 1e-4, "Absolute XTE is 2.0m");

    // West side test: U = (-2.0, -5.0), heading North (0.0)
    // Vector from user to trail: e = Q - U = (+2.0, 0.0) (points East).
    // e_perp = (+2.0)*(-1.0) = -2.0m.
    // Trail is to user's RIGHT -> xte must be NEGATIVE (-).
    GuidanceOut gWest = nav.update({-2.0, -5.0}, 0.0, 1.0);
    std::cout << "  User on West of Northbound trail: XTE = " << gWest.xte << " m" << std::endl;
    TEST_ASSERT(gWest.xte > -2.1 && gWest.xte < -1.9, "User on West side: XTE is NEGATIVE (-2.0m), trail is to user's RIGHT");
    TEST_ASSERT(std::abs(gWest.absXte - 2.0) < 1e-4, "Absolute XTE is 2.0m");
}

// 10. testHeadingError: Steering error and angle wrapping with 180 deg tie-break
void testHeadingError() {
    std::cout << "\nTest 10: Steering Error & 180° Tie-Break" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 10}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // User at (0, 9) heading South (-pi rad): desired heading is South (-pi rad).
    // Steering error should be ~0.
    GuidanceOut gAligned = nav.update({0.0, 9.0}, -M_PI, 1.0);
    TEST_ASSERT(std::abs(gAligned.steeringErr) < 0.1, "Steering error is ~0 rad when aligned with reverse path");

    // User heading North (0.0 rad): desired heading is South (pi or -pi).
    // Exactly 180° opposite -> tie-break policy fires (+pi = turn RIGHT)
    GuidanceOut gOpposite = nav.update({0.0, 9.0}, 0.0, 1.0);
    TEST_ASSERT(std::abs(std::abs(gOpposite.steeringErr) - M_PI) < 0.1, "180° steering error detected and wrapped");
    TEST_ASSERT(gOpposite.steeringErr > 0.0, "Deterministic 180° tie-break defaults to turn RIGHT (+pi)");
}

// 11. testCarrotLookAhead: Carrot placed ahead along reverse trail
void testCarrotLookAhead() {
    std::cout << "\nTest 11: Carrot Look-Ahead" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // User at (0, 18), speed 1.0 m/s -> lookAheadT = 2.0s -> lookAhead = 2.0m
    // Projected Q is (0, 18). Moving South along trail -> Carrot should be at (0, 16).
    GuidanceOut g = nav.update({0.0, 18.0}, -M_PI, 1.0);
    TEST_ASSERT(std::abs(g.carrotPoint.e - 0.0) < 1e-4, "Carrot East is 0.0");
    TEST_ASSERT(std::abs(g.carrotPoint.n - 16.0) < 0.2, "Carrot North is placed ~2.0m ahead at North=16.0m");
}

// 12. testCarrotAcrossSegments: Carrot correctly crosses breadcrumb segment boundaries
void testCarrotAcrossSegments() {
    std::cout << "\nTest 12: Carrot Across Segment Boundaries" << std::endl;
    // L-shaped trail: (0,0) -> (0,10) -> (10,10)
    // Reverse: (10,10) -> (0,10) (segment 1: West) then (0,10) -> (0,0) (segment 2: South)
    std::vector<P2> wps = {{0, 0}, {0, 10}, {10, 10}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Place user at (1.0, 10.0), 1 meter before corner (0,10).
    // Look-ahead distance is 3.0m. Carrot must turn the corner and be at (0, 8.0) (South)!
    GuidanceOut g = nav.update({1.0, 10.0}, -M_PI / 2.0, 1.5);
    std::cout << "  Corner at (0,10). User at (1,10). Carrot at: (" << g.carrotPoint.e << ", " << g.carrotPoint.n << ")" << std::endl;
    TEST_ASSERT(g.carrotPoint.e < 0.1, "Carrot crossed corner onto South segment (East ~0)");
    TEST_ASSERT(g.carrotPoint.n < 10.0, "Carrot progressed South of corner (North < 10.0)");
}

// 13. testMatcherHysteresis: Small cost differences do not cause segment switching
void testMatcherHysteresis() {
    std::cout << "\nTest 13: Matcher Hysteresis" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 10}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    GuidanceOut g1 = nav.update({0.0, 9.0}, -M_PI, 1.0);
    int seg1 = g1.segIndex;

    // Small shift of 0.05m along path: delta J is far below dJswitch (0.15)
    GuidanceOut g2 = nav.update({0.0, 8.95}, -M_PI, 1.0);
    TEST_ASSERT(g2.segIndex == seg1, "Hysteresis prevents segment chatter on minor position jitter");
}

// 14. testSelfCrossingDisambiguation: Critical self-crossing test!
void testSelfCrossingDisambiguation() {
    std::cout << "\nTest 14: Critical Self-Crossing Disambiguation" << std::endl;
    // Figure-eight / crossing path:
    // P0: (0, 0)
    // Leg 1: (0, 0) -> (0, 10)
    // Leg 2: (0, 10) -> (10, 10)
    // Leg 3: (10, 10) -> (0, 0) [Crosses origin P0!]
    // Leg 4: (0, 0) -> (-10, -10)
    std::vector<P2> wps = {{0, 0}, {0, 10}, {10, 10}, {0, 0}, {-10, -10}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // During reverse navigation from (-10, -10):
    // Reverse route first reaches (0, 0) along Leg 4 (segment near start of return).
    // The user passing through (0, 0) MUST NOT match Leg 1 (which also connects to (0,0)),
    // because Leg 1 is at the very END of the return journey near P0!
    GuidanceOut g = nav.update({-1.0, -1.0}, M_PI / 4.0, 1.0); // Heading towards (0,0) from (-10,-10)
    std::cout << "  At self-crossing near (0,0): Matched segment = " << g.segIndex
              << " | Total segments = " << nav.getPointCount() - 1 << std::endl;

    TEST_ASSERT(g.segIndex < 15, "Matcher selects the early reverse segment (Leg 4), NOT the distant Leg 1 at P0");
}

// 15. testLoopProgress: Loop does not jump to wrong occurrence of same location
void testLoopProgress() {
    std::cout << "\nTest 15: Loop Progress Preservation" << std::endl;
    // Square loop: (0,0) -> (0,10) -> (10,10) -> (10,0) -> (0,0) -> (0,-10)
    std::vector<P2> wps = {{0, 0}, {0, 10}, {10, 10}, {10, 0}, {0, 0}, {0, -10}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Initial return starts at (0, -10) heading to (0, 0)
    GuidanceOut g1 = nav.update({0.0, -8.0}, 0.0, 1.0);
    int initialSeg = g1.segIndex;
    TEST_ASSERT(initialSeg <= 3, "Starts on the correct initial leg of the loop");

    // As user progresses past (0,0), progress index must increase along loop, not teleport to P0
    GuidanceOut g2 = nav.update({2.0, 0.0}, M_PI / 2.0, 1.0); // along (0,0)->(10,0) in reverse
    TEST_ASSERT(g2.segIndex >= initialSeg, "Progress moves forward into the loop segments without jumping");
}

// 16. testReturnComplete: Approaching P0 produces RETURN_COMPLETE
void testReturnComplete() {
    std::cout << "\nTest 16: Return Complete Arrival Detection" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 5}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Step forward along reverse trail toward P0 (0, 0)
    nav.update({0.0, 4.0}, -M_PI, 1.0);
    nav.update({0.0, 3.0}, -M_PI, 1.0);
    nav.update({0.0, 2.0}, -M_PI, 1.0);
    nav.update({0.0, 1.0}, -M_PI, 1.0);

    // User at (0.0, 0.5): distance to P0 is 0.5m (inside rArrive = 3.0m).
    // Test persistence updates (3 consecutive updates required)
    GuidanceOut gA = nav.update({0.0, 0.5}, -M_PI, 1.0);
    GuidanceOut gB = nav.update({0.0, 0.3}, -M_PI, 1.0);
    GuidanceOut gC = nav.update({0.0, 0.1}, -M_PI, 1.0);

    TEST_ASSERT(gC.returnComplete, "Return complete triggered after persistent arrival at P0");
    TEST_ASSERT(gC.state == NavigationState::RETURN_COMPLETE, "Navigation state transitioned to RETURN_COMPLETE");
}

// 17. testDeterministicReplay: Identical inputs produce identical outputs
void testDeterministicReplay() {
    std::cout << "\nTest 17: Deterministic Replay Reproducibility" << std::endl;
    std::vector<P2> wps = {{0, 0}, {5, 5}, {10, 0}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav1, nav2;
    nav1.startReturn(trail);
    nav2.startReturn(trail);

    std::vector<P2> testPositions = {{9.0, 1.0}, {8.0, 2.0}, {6.0, 4.0}, {5.0, 5.0}, {3.0, 3.0}, {1.0, 1.0}};
    for (const auto& p : testPositions) {
        GuidanceOut out1 = nav1.update(p, -2.0, 1.2);
        GuidanceOut out2 = nav2.update(p, -2.0, 1.2);

        TEST_ASSERT(out1.segIndex == out2.segIndex, "Replay: matched segment identical");
        TEST_ASSERT(std::abs(out1.xte - out2.xte) < 1e-6, "Replay: XTE strictly identical");
        TEST_ASSERT(std::abs(out1.steeringErr - out2.steeringErr) < 1e-6, "Replay: steering error strictly identical");
        TEST_ASSERT(std::abs(out1.carrotPoint.e - out2.carrotPoint.e) < 1e-6 &&
                    std::abs(out1.carrotPoint.n - out2.carrotPoint.n) < 1e-6, "Replay: carrot strictly identical");
        TEST_ASSERT(out1.state == out2.state, "Replay: state strictly identical");
    }
}

// 18. testMatchingCostDiagnostics: Clarification 2 report
void testMatchingCostDiagnostics() {
    std::cout << "\nTest 18: Matching Cost Diagnostics Report (Clarification 2)" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 10}};
    TrailMemory trail = buildTrail(wps, 1.0f, 0.90f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Place user at East=1.5m, North=8.5m, heading -pi/2 (West), speed 1.0
    GuidanceOut g = nav.update({1.5, 8.5}, -M_PI / 2.0, 1.0);

    std::cout << "  Representative Cost Component Breakdown:" << std::endl;
    std::cout << "    - J_distance   = " << std::fixed << std::setprecision(4) << g.jDist << std::endl;
    std::cout << "    - J_sequence   = " << g.jSeq << std::endl;
    std::cout << "    - J_heading    = " << g.jHead << std::endl;
    std::cout << "    - J_confidence = " << g.jConf << std::endl;
    std::cout << "    ------------------------------------" << std::endl;
    std::cout << "    - J_total      = " << g.jTotal << std::endl;

    TEST_ASSERT(g.jDist > 0.0, "J_distance contribution positive and non-zero");
    TEST_ASSERT(g.jTotal > 0.0, "Total cost positive");
    TEST_ASSERT(std::abs(g.jTotal - (g.jDist + g.jSeq + g.jHead + g.jConf)) < 1e-5, "Total cost equals sum of terms");
}

int main() {
    std::cout << "========================================================" << std::endl;
    std::cout << "STARTING M5 DETERMINISTIC REVERSE NAVIGATION UNIT TESTS" << std::endl;
    std::cout << "========================================================" << std::endl;

    testReverseOrdering();
    testReversePointAccessor();
    testReverseHeading();
    testSegmentProjection();
    testZeroLengthSegment();
    testStraightTrailMatching();
    testProgressMonotonicity();
    testProgressWindow();
    testCrossTrackError();
    testHeadingError();
    testCarrotLookAhead();
    testCarrotAcrossSegments();
    testMatcherHysteresis();
    testSelfCrossingDisambiguation();
    testLoopProgress();
    testReturnComplete();
    testDeterministicReplay();
    testMatchingCostDiagnostics();

    std::cout << "========================================================" << std::endl;
    std::cout << "ALL 18 M5 DETERMINISTIC TESTS PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "========================================================" << std::endl;
    return 0;
}
