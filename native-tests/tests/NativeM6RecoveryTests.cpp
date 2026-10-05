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

// Helper to build synthetic trail
static TrailMemory buildTrail(const std::vector<P2>& waypoints, float stepLen = 1.0f, float cMag = 1.0f) {
    TrailMemory trail;
    int64_t t = 1000000000LL;
    trail.startRecording(t, 0.0f); // P0 at waypoints[0]

    uint32_t step = 0;
    float cumDist = 0.0f;
    P2 prev = waypoints[0];

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

// 1. testDeviationTransition: ON_TRAIL -> DEVIATING when |XTE| > 3m
void testDeviationTransition() {
    std::cout << "\nTest 1: Deviation Transition (ON_TRAIL -> DEVIATING)" << std::endl;
    // Trail from (0,0) to (0,20). Return is Southbound from (0,20) to (0,0).
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // User on trail at (0, 18), heading South (pi)
    auto g1 = nav.update({0.0, 18.0}, M_PI);
    TEST_ASSERT(g1.state == NavigationState::ON_TRAIL, "User on trail is ON_TRAIL");
    TEST_ASSERT(g1.guidanceMode == GuidanceMode::NORMAL_CARROT, "ON_TRAIL uses NORMAL_CARROT mode");
    TEST_ASSERT(std::abs(g1.xte) < 0.1, "XTE is approximately zero");

    // Move laterally to 3.5m East (3.0m < |XTE| <= 5.0m)
    auto g2 = nav.update({3.5, 18.0}, M_PI);
    TEST_ASSERT(g2.state == NavigationState::DEVIATING, "3.5m lateral offset transitions to DEVIATING");
    TEST_ASSERT(g2.guidanceMode == GuidanceMode::LATERAL_BLEND, "DEVIATING uses LATERAL_BLEND mode");
    TEST_ASSERT(std::abs(g2.absXte - 3.5) < 1e-3, "absXte matches lateral offset 3.5m");
}

// 2. testOffTrailPersistence: DEVIATING -> OFF_TRAIL only after |XTE| > 5m for 3 consecutive updates
void testOffTrailPersistence() {
    std::cout << "\nTest 2: Off-Trail Persistence Debounce (3 consecutive updates)" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Initial on trail
    nav.update({0.0, 18.0}, M_PI);

    // Move > 5.0m away to East (e=6.0m)
    // Update 1: offTrailCounter = 1 < 3 -> should be DEVIATING, NOT OFF_TRAIL
    auto g1 = nav.update({6.0, 18.0}, M_PI);
    TEST_ASSERT(g1.state == NavigationState::DEVIATING, "Update 1 (>5m) stays DEVIATING during debounce");
    TEST_ASSERT(g1.guidanceMode == GuidanceMode::LATERAL_BLEND, "Update 1 remains LATERAL_BLEND");

    // Update 2: offTrailCounter = 2 < 3 -> still DEVIATING
    auto g2 = nav.update({6.0, 18.0}, M_PI);
    TEST_ASSERT(g2.state == NavigationState::DEVIATING, "Update 2 (>5m) stays DEVIATING during debounce");

    // Update 3: offTrailCounter = 3 >= 3 -> transitions to OFF_TRAIL
    auto g3 = nav.update({6.0, 18.0}, M_PI);
    TEST_ASSERT(g3.state == NavigationState::OFF_TRAIL, "Update 3 (>5m) transitions to OFF_TRAIL");
    TEST_ASSERT(g3.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "OFF_TRAIL selects DIRECT_RECOVERY");
    TEST_ASSERT(g3.recoveryActive, "recoveryActive is true in OFF_TRAIL");
}

// 3. testRecoveryThresholdHysteresis: OFF_TRAIL -> RECOVERING (<=5m), stays RECOVERING until <=2m
void testRecoveryThresholdHysteresis() {
    std::cout << "\nTest 3: Recovery Threshold Hysteresis (5m > XTE > 2m does NOT exit RECOVERING)" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Push into OFF_TRAIL (3 updates at 6.0m)
    nav.update({6.0, 18.0}, M_PI);
    nav.update({6.0, 18.0}, M_PI);
    auto gOff = nav.update({6.0, 18.0}, M_PI);
    TEST_ASSERT(gOff.state == NavigationState::OFF_TRAIL, "Confirmed OFF_TRAIL state");

    // Move back inside 5.0m gating to 4.0m
    auto gRecov1 = nav.update({4.0, 18.0}, M_PI);
    TEST_ASSERT(gRecov1.state == NavigationState::RECOVERING, "Re-entering 5.0m gate transitions to RECOVERING");
    TEST_ASSERT(gRecov1.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "RECOVERING uses DIRECT_RECOVERY");
    TEST_ASSERT(gRecov1.recoveryActive, "recoveryActive is true in RECOVERING");

    // Move to 2.5m (which is 5.0m > XTE > 2.0m)
    auto gRecov2 = nav.update({2.5, 18.0}, M_PI);
    TEST_ASSERT(gRecov2.state == NavigationState::RECOVERING, "XTE=2.5m (>2m) STAYS in RECOVERING (does NOT prematurely return to ON_TRAIL)");
    TEST_ASSERT(gRecov2.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "XTE=2.5m continues DIRECT_RECOVERY");

    // Move to 1.8m (<= 2.0m recovery threshold)
    auto gOn = nav.update({1.8, 18.0}, M_PI);
    TEST_ASSERT(gOn.state == NavigationState::ON_TRAIL, "XTE=1.8m (<=2m) successfully transitions to ON_TRAIL");
    TEST_ASSERT(gOn.guidanceMode == GuidanceMode::NORMAL_CARROT, "ON_TRAIL restores NORMAL_CARROT");
    TEST_ASSERT(!gOn.recoveryActive, "recoveryActive is false once ON_TRAIL");

    // Now test moving back up to 2.5m while already ON_TRAIL: 2.5m <= 3.0m deviation threshold -> stays ON_TRAIL!
    auto gOn2 = nav.update({2.5, 18.0}, M_PI);
    TEST_ASSERT(gOn2.state == NavigationState::ON_TRAIL, "Already ON_TRAIL at 2.5m stays ON_TRAIL (hysteresis confirmed)");
}

// 4. testRecoveryVectorPointsTowardTrail: Orthogonal recovery toward trail Q in OFF_TRAIL
void testRecoveryVectorPointsTowardTrail() {
    std::cout << "\nTest 4: Recovery Vector Points Directly Toward Trail Q" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 30}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // User follows trail down to (0, 25)
    nav.update({0.0, 29.0}, M_PI);
    nav.update({0.0, 27.0}, M_PI);
    nav.update({0.0, 25.0}, M_PI);

    // Now user deviates laterally to (7.0, 25.0) for 3 updates -> OFF_TRAIL
    nav.update({7.0, 25.0}, M_PI);
    nav.update({7.0, 25.0}, M_PI);
    auto g = nav.update({7.0, 25.0}, M_PI);

    TEST_ASSERT(g.state == NavigationState::OFF_TRAIL, "State is OFF_TRAIL");
    TEST_ASSERT(g.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "Mode is DIRECT_RECOVERY");

    // Trail segment is along East=0, active segment at North ~25.
    // Q should be at (0.0, 25.0).
    TEST_ASSERT(std::abs(g.projPoint.e - 0.0) < 1e-2, "Q.e is 0.0");
    TEST_ASSERT(std::abs(g.projPoint.n - 25.0) < 1e-2, "Q.n is 25.0");
    TEST_ASSERT(std::abs(g.recoveryTarget.e - 0.0) < 1e-2, "recoveryTarget is Q");

    // Guidance vector g = normalize(Q - U) = normalize((0, 25) - (7, 25)) = (-1.0, 0.0) [due West]
    TEST_ASSERT(std::abs(g.guidanceVector.e - (-1.0)) < 1e-3, "Guidance vector East is -1.0 (West directly toward trail)");
    TEST_ASSERT(std::abs(g.guidanceVector.n - 0.0) < 1e-3, "Guidance vector North is 0.0");
}

// 5. testRecoveryVectorEastOfTrail: User east of trail -> recovery vector points West
void testRecoveryVectorEastOfTrail() {
    std::cout << "\nTest 5: Recovery Vector East of Trail" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    nav.update({0.0, 18.0}, M_PI);
    nav.update({0.0, 16.0}, M_PI);

    for (int i = 0; i < 3; ++i) nav.update({6.0, 16.0}, M_PI);
    auto g = nav.getLatestGuidance();

    TEST_ASSERT(g.guidanceVector.e < -0.9, "User East of trail has Westbound recovery vector (g.e < 0)");
    TEST_ASSERT(std::abs(g.guidanceVector.n) < 0.1, "User orthogonal to trail has near-zero North component");
}

// 6. testRecoveryVectorWestOfTrail: User west of trail -> recovery vector points East
void testRecoveryVectorWestOfTrail() {
    std::cout << "\nTest 6: Recovery Vector West of Trail" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    nav.update({0.0, 18.0}, M_PI);
    nav.update({0.0, 16.0}, M_PI);

    for (int i = 0; i < 3; ++i) nav.update({-6.0, 16.0}, M_PI);
    auto g = nav.getLatestGuidance();

    TEST_ASSERT(g.guidanceVector.e > 0.9, "User West of trail has Eastbound recovery vector (g.e > 0)");
    TEST_ASSERT(std::abs(g.guidanceVector.n) < 0.1, "User orthogonal to trail has near-zero North component");
}

// 7. testLateralBlendEastSide: Northbound reverse trail, user 2m East of trail
void testLateralBlendEastSide() {
    std::cout << "\nTest 7: Lateral Blend Sign & Direction (Test A: User 2m East of Northbound Reverse Trail)" << std::endl;
    // Outward trail: (0, 20) -> (0, 0).
    // Reverse trail: (0, 0) -> (0, 20), which runs Northbound!
    std::vector<P2> wps = {{0, 20}, {0, 0}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Force segment to point North: reverse goes from P1 (0,0) to P0 (0,20).
    // Reverse tangent t = (0, 1) (North).
    // Left normal nLeft = [-tN, tE] = [-1, 0] (West).
    // User at (2.0, 10.0), 2m East of trail. User heading North (0 rad).
    auto g = nav.update({2.0, 10.0}, 0.0);

    TEST_ASSERT(g.xte > 0.0, "e_perp > 0 when user is 2m East of Northbound trail (trail is on LEFT)");
    std::cout << "  Diagnostic: XTE=" << g.xte << "m, g=(" << g.guidanceVector.e << ", " << g.guidanceVector.n << ")" << std::endl;

    // Lateral blend equation: g = normalize(t + clamp(ke * xte) * nLeft)
    // t=(0, 1), nLeft=(-1, 0), xte=+2.0, ke=0.25 -> lat=+0.5.
    // gb = (0, 1) + 0.5 * (-1, 0) = (-0.5, 1.0).
    // Normalized: g = (-0.447, 0.894).
    TEST_ASSERT(g.guidanceVector.e < 0.0, "Lateral component points WEST (negative East) toward trail");
    TEST_ASSERT(g.guidanceVector.n > 0.0, "Forward reverse component points NORTH along reverse trail");
    TEST_ASSERT(std::abs(norm(g.guidanceVector) - 1.0) < 1e-4, "Guidance vector is properly normalized");
}

// 8. testLateralBlendWestSide: Northbound reverse trail, user 2m West of trail
void testLateralBlendWestSide() {
    std::cout << "\nTest 8: Lateral Blend Sign & Direction (Test B: User 2m West of Northbound Reverse Trail)" << std::endl;
    std::vector<P2> wps = {{0, 20}, {0, 0}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // User at (-2.0, 10.0), 2m West of trail. User heading North (0 rad).
    auto g = nav.update({-2.0, 10.0}, 0.0);

    TEST_ASSERT(g.xte < 0.0, "e_perp < 0 when user is 2m West of Northbound trail (trail is on RIGHT)");
    std::cout << "  Diagnostic: XTE=" << g.xte << "m, g=(" << g.guidanceVector.e << ", " << g.guidanceVector.n << ")" << std::endl;

    // t=(0, 1), nLeft=(-1, 0), xte=-2.0, ke=0.25 -> lat=-0.5.
    // gb = (0, 1) + (-0.5) * (-1, 0) = (+0.5, 1.0).
    // Normalized: g = (+0.447, 0.894).
    TEST_ASSERT(g.guidanceVector.e > 0.0, "Lateral component points EAST (positive East) toward trail");
    TEST_ASSERT(g.guidanceVector.n > 0.0, "Forward reverse component points NORTH along reverse trail");
    TEST_ASSERT(std::abs(norm(g.guidanceVector) - 1.0) < 1e-4, "Guidance vector is properly normalized");
}

// 9. testGuidanceModeSelection: Verify all 5 state -> mode mappings
void testGuidanceModeSelection() {
    std::cout << "\nTest 9: Guidance Mode Explicit Mapping" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 10}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // ON_TRAIL -> NORMAL_CARROT
    auto g1 = nav.update({0.0, 8.0}, M_PI);
    TEST_ASSERT(g1.state == NavigationState::ON_TRAIL, "State ON_TRAIL");
    TEST_ASSERT(g1.guidanceMode == GuidanceMode::NORMAL_CARROT, "ON_TRAIL -> NORMAL_CARROT");

    // DEVIATING -> LATERAL_BLEND
    auto g2 = nav.update({3.5, 8.0}, M_PI);
    TEST_ASSERT(g2.state == NavigationState::DEVIATING, "State DEVIATING");
    TEST_ASSERT(g2.guidanceMode == GuidanceMode::LATERAL_BLEND, "DEVIATING -> LATERAL_BLEND");

    // OFF_TRAIL -> DIRECT_RECOVERY
    nav.update({6.0, 8.0}, M_PI);
    nav.update({6.0, 8.0}, M_PI);
    auto g3 = nav.update({6.0, 8.0}, M_PI);
    TEST_ASSERT(g3.state == NavigationState::OFF_TRAIL, "State OFF_TRAIL");
    TEST_ASSERT(g3.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "OFF_TRAIL -> DIRECT_RECOVERY");

    // RECOVERING -> DIRECT_RECOVERY
    auto g4 = nav.update({3.0, 8.0}, M_PI);
    TEST_ASSERT(g4.state == NavigationState::RECOVERING, "State RECOVERING");
    TEST_ASSERT(g4.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "RECOVERING -> DIRECT_RECOVERY");

    // RETURN_COMPLETE -> ARRIVED
    for (int i = 0; i < 5; ++i) nav.update({0.0, 0.0}, M_PI);
    auto g5 = nav.getLatestGuidance();
    TEST_ASSERT(g5.state == NavigationState::RETURN_COMPLETE, "State RETURN_COMPLETE");
    TEST_ASSERT(g5.guidanceMode == GuidanceMode::ARRIVED, "RETURN_COMPLETE -> ARRIVED");
    TEST_ASSERT(g5.guidanceVector.e == 0.0 && g5.guidanceVector.n == 0.0, "ARRIVED has zero guidance vector");
}

// 10. testTurnStraight: |DeltaPsi| < 15 deg
void testTurnStraight() {
    std::cout << "\nTest 10: TurnCommand STRAIGHT" << std::endl;
    constexpr double DEG2RAD = M_PI / 180.0;
    TEST_ASSERT(classifyTurnCommand(0.0) == TurnCommand::STRAIGHT, "0 deg is STRAIGHT");
    TEST_ASSERT(classifyTurnCommand(10.0 * DEG2RAD) == TurnCommand::STRAIGHT, "+10 deg is STRAIGHT");
    TEST_ASSERT(classifyTurnCommand(-10.0 * DEG2RAD) == TurnCommand::STRAIGHT, "-10 deg is STRAIGHT");
    TEST_ASSERT(classifyTurnCommand(14.99 * DEG2RAD) == TurnCommand::STRAIGHT, "+14.99 deg is STRAIGHT");
    TEST_ASSERT(classifyTurnCommand(-14.99 * DEG2RAD) == TurnCommand::STRAIGHT, "-14.99 deg is STRAIGHT");
}

// 11. testTurnLeft: -165 deg <= DeltaPsi <= -15 deg
void testTurnLeft() {
    std::cout << "\nTest 11: TurnCommand TURN_LEFT" << std::endl;
    constexpr double DEG2RAD = M_PI / 180.0;
    TEST_ASSERT(classifyTurnCommand(-15.0 * DEG2RAD) == TurnCommand::TURN_LEFT, "-15 deg is TURN_LEFT");
    TEST_ASSERT(classifyTurnCommand(-45.0 * DEG2RAD) == TurnCommand::TURN_LEFT, "-45 deg is TURN_LEFT");
    TEST_ASSERT(classifyTurnCommand(-90.0 * DEG2RAD) == TurnCommand::TURN_LEFT, "-90 deg is TURN_LEFT");
    TEST_ASSERT(classifyTurnCommand(-165.0 * DEG2RAD) == TurnCommand::TURN_LEFT, "-165 deg is TURN_LEFT");
}

// 12. testTurnRight: +15 deg <= DeltaPsi <= +165 deg
void testTurnRight() {
    std::cout << "\nTest 12: TurnCommand TURN_RIGHT" << std::endl;
    constexpr double DEG2RAD = M_PI / 180.0;
    TEST_ASSERT(classifyTurnCommand(+15.0 * DEG2RAD) == TurnCommand::TURN_RIGHT, "+15 deg is TURN_RIGHT");
    TEST_ASSERT(classifyTurnCommand(+45.0 * DEG2RAD) == TurnCommand::TURN_RIGHT, "+45 deg is TURN_RIGHT");
    TEST_ASSERT(classifyTurnCommand(+90.0 * DEG2RAD) == TurnCommand::TURN_RIGHT, "+90 deg is TURN_RIGHT");
    TEST_ASSERT(classifyTurnCommand(+165.0 * DEG2RAD) == TurnCommand::TURN_RIGHT, "+165 deg is TURN_RIGHT");
}

// 13. testUTurn: |DeltaPsi| > 165 deg
void testUTurn() {
    std::cout << "\nTest 13: TurnCommand U_TURN" << std::endl;
    constexpr double DEG2RAD = M_PI / 180.0;
    TEST_ASSERT(classifyTurnCommand(+165.01 * DEG2RAD) == TurnCommand::U_TURN, "+165.01 deg is U_TURN");
    TEST_ASSERT(classifyTurnCommand(-165.01 * DEG2RAD) == TurnCommand::U_TURN, "-165.01 deg is U_TURN");
    TEST_ASSERT(classifyTurnCommand(+180.0 * DEG2RAD) == TurnCommand::U_TURN, "+180 deg is U_TURN");
    TEST_ASSERT(classifyTurnCommand(-180.0 * DEG2RAD) == TurnCommand::U_TURN, "-180 deg is U_TURN");
}

// 14. testTurnBoundaryValues: All explicit required boundary values
void testTurnBoundaryValues() {
    std::cout << "\nTest 14: Turn Command Explicit Boundary Values" << std::endl;
    constexpr double DEG2RAD = M_PI / 180.0;

    TEST_ASSERT(classifyTurnCommand(0.0 * DEG2RAD) == TurnCommand::STRAIGHT, "0 deg -> STRAIGHT");
    TEST_ASSERT(classifyTurnCommand(14.9 * DEG2RAD) == TurnCommand::STRAIGHT, "+14.9 deg -> STRAIGHT");
    TEST_ASSERT(classifyTurnCommand(15.0 * DEG2RAD) == TurnCommand::TURN_RIGHT, "+15.0 deg -> TURN_RIGHT");
    TEST_ASSERT(classifyTurnCommand(165.0 * DEG2RAD) == TurnCommand::TURN_RIGHT, "+165.0 deg -> TURN_RIGHT");
    TEST_ASSERT(classifyTurnCommand(165.1 * DEG2RAD) == TurnCommand::U_TURN, "+165.1 deg -> U_TURN");

    TEST_ASSERT(classifyTurnCommand(-14.9 * DEG2RAD) == TurnCommand::STRAIGHT, "-14.9 deg -> STRAIGHT");
    TEST_ASSERT(classifyTurnCommand(-15.0 * DEG2RAD) == TurnCommand::TURN_LEFT, "-15.0 deg -> TURN_LEFT");
    TEST_ASSERT(classifyTurnCommand(-165.0 * DEG2RAD) == TurnCommand::TURN_LEFT, "-165.0 deg -> TURN_LEFT");
    TEST_ASSERT(classifyTurnCommand(-165.1 * DEG2RAD) == TurnCommand::U_TURN, "-165.1 deg -> U_TURN");

    TEST_ASSERT(classifyTurnCommand(+180.0 * DEG2RAD) == TurnCommand::U_TURN, "+180.0 deg -> U_TURN");
    TEST_ASSERT(classifyTurnCommand(-180.0 * DEG2RAD) == TurnCommand::U_TURN, "-180.0 deg -> U_TURN");
}

// 15. testExisting180DegreeTieBreak: Preserve M5 180° tie-break and classify as U_TURN
void testExisting180DegreeTieBreak() {
    std::cout << "\nTest 15: Existing 180° Tie-Break Policy Preservation" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 20}};
    TrailMemory trail = buildTrail(wps);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Reverse carrot is South (bearing pi).
    // If user is facing North (0.0 rad), steering error is exactly pi (180 deg).
    // Initial prevTurn_ is +1 -> resolves deterministically to +pi.
    auto g = nav.update({0.0, 18.0}, 0.0);

    TEST_ASSERT(std::abs(g.steeringErr - M_PI) < 1e-4, "180 deg tie-break resolves deterministically to +M_PI");
    TEST_ASSERT(g.turnCommand == TurnCommand::U_TURN, "Resolved +M_PI classifies as U_TURN");
}

// 16. testSelfCrossingRecovery: Sequence-aware matching at self-crossing
void testSelfCrossingRecovery() {
    std::cout << "\nTest 16: Self-Crossing Recovery (Sequence Awareness)" << std::endl;
    // Build a loop trail that crosses itself:
    // P0: (0, 0)
    // P1: (0, 15)
    // P2: (15, 15)
    // P3: (15, 0)
    // P4: (0.1, 0.1) [Geographically identical to P0 within 0.14m!]
    std::vector<P2> wps = {{0, 0}, {0, 15}, {15, 15}, {15, 0}, {0.1, 0.1}, {0, -10}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Return starts at P5 (0, -10).
    // Segment 0 connects P5 (0, -10) to P4 (0.1, 0.1) (10 steps).
    // User is at (0.1, -1.0) heading North toward P4.
    // Euclidean distance to P0 (0,0) is ~1.0m.
    // Euclidean distance to P4 (0.1, 0.1) is ~1.1m.
    // Progress pointer must stay on reverse segment 0..9 (P5 -> P4), NOT jump across the entire journey to P0 (>55)!
    auto g = nav.update({0.1, -1.0}, 0.0);

    TEST_ASSERT(g.segIndex <= 10, "Matcher selects active recent reverse segment near P4, NOT beginning P0");
    TEST_ASSERT(g.state == NavigationState::ON_TRAIL, "User is ON_TRAIL on correct sequence branch");
}

// 17. testRecoveryProgressPreservation: Progress pointer preserved during deviation & recovery
void testRecoveryProgressPreservation() {
    std::cout << "\nTest 17: Progress Pointer Preservation During Off-Trail & Recovery" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 15}, {15, 15}, {30, 15}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    nav.startReturn(trail);

    // Move along reverse trail to intermediate segment
    auto g1 = nav.update({25.0, 15.0}, -M_PI / 2.0);
    int progBefore = g1.reverseProgressIndex;

    // Deviate 10m off trail into OFF_TRAIL
    for (int i = 0; i < 3; ++i) {
        nav.update({25.0, 25.0}, 0.0);
    }
    auto gOff = nav.getLatestGuidance();
    TEST_ASSERT(gOff.state == NavigationState::OFF_TRAIL, "User is OFF_TRAIL");

    // Return back toward trail: N=18.0 (d=3.0m, within 5m gate but > 2m recov threshold)
    auto gRecov = nav.update({25.0, 18.0}, -M_PI / 2.0);
    TEST_ASSERT(gRecov.state == NavigationState::RECOVERING, "User is RECOVERING at 3.0m offset");
    TEST_ASSERT(std::abs(gRecov.reverseProgressIndex - progBefore) <= 1, "Reverse progress index preserved within search window");

    // Move inside 2m: N=16.0 (d=1.0m <= 2.0m)
    auto gOn = nav.update({25.0, 16.0}, -M_PI / 2.0);
    TEST_ASSERT(gOn.state == NavigationState::ON_TRAIL, "User transitions to ON_TRAIL at 1.0m offset");
}

// 18. testDeterministicRecoveryReplay: Byte-for-byte identical state sequence
void testDeterministicRecoveryReplay() {
    std::cout << "\nTest 18: Deterministic Recovery Replay" << std::endl;
    std::vector<P2> wps = {{0, 0}, {0, 25}, {25, 25}, {25, 50}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    std::vector<P2> userTrajectory = {
        {25.0, 45.0}, {25.0, 40.0}, {28.5, 35.0}, {32.0, 35.0},
        {32.0, 35.0}, {32.0, 35.0}, {29.0, 35.0}, {26.5, 35.0},
        {25.0, 30.0}, {25.0, 25.0}
    };

    auto runScenario = [&](const TrailMemory& tr) {
        ReverseNavigator nav;
        nav.startReturn(tr);
        std::vector<GuidanceOut> trace;
        for (const auto& u : userTrajectory) {
            trace.push_back(nav.update(u, M_PI));
        }
        return trace;
    };

    auto trace1 = runScenario(trail);
    auto trace2 = runScenario(trail);

    TEST_ASSERT(trace1.size() == trace2.size(), "Traces have identical length");
    for (size_t i = 0; i < trace1.size(); ++i) {
        TEST_ASSERT(trace1[i].state == trace2[i].state, "State codes match exactly");
        TEST_ASSERT(trace1[i].guidanceMode == trace2[i].guidanceMode, "Guidance modes match exactly");
        TEST_ASSERT(trace1[i].turnCommand == trace2[i].turnCommand, "Turn commands match exactly");
        TEST_ASSERT(std::abs(trace1[i].xte - trace2[i].xte) < 1e-9, "XTE matches to float precision");
        TEST_ASSERT(std::abs(trace1[i].steeringErr - trace2[i].steeringErr) < 1e-9, "Steering error matches to float precision");
        TEST_ASSERT(std::abs(trace1[i].guidanceVector.e - trace2[i].guidanceVector.e) < 1e-9, "Guidance vector East matches exactly");
        TEST_ASSERT(std::abs(trace1[i].guidanceVector.n - trace2[i].guidanceVector.n) < 1e-9, "Guidance vector North matches exactly");
    }
}

// 19. testEndToEndSyntheticScenario: Mandatory scenario from Section 11
void testEndToEndSyntheticScenario() {
    std::cout << "\nTest 19: End-To-End Synthetic Guidance & Recovery Scenario (§11)" << std::endl;
    // Trail: P0 (0,0) -> P5 (0,25) -> P10 (25,25) -> P15 (25,50)
    std::vector<P2> wps = {{0, 0}, {0, 25}, {25, 25}, {25, 50}};
    TrailMemory trail = buildTrail(wps, 1.0f);

    ReverseNavigator nav;
    bool started = nav.startReturn(trail);
    TEST_ASSERT(started, "Return started at P15");

    // Phase 1: Follow trail South along segment from (25,50) to (25,25)
    std::cout << "--- Phase 1: Follow trail ---" << std::endl;
    auto g1 = nav.update({25.0, 45.0}, M_PI);
    TEST_ASSERT(g1.state == NavigationState::ON_TRAIL, "Phase 1: State is ON_TRAIL");
    TEST_ASSERT(g1.guidanceMode == GuidanceMode::NORMAL_CARROT, "Phase 1: GuidanceMode is NORMAL_CARROT");
    TEST_ASSERT(g1.turnCommand == TurnCommand::STRAIGHT, "Phase 1: TurnCommand is STRAIGHT");

    // Phase 2: Move 3.5m laterally East (e=28.5, n=35.0).
    // Reverse trail is running South: t = (0, -1).
    // Left normal: nLeft = [-tN, tE] = [+1, 0] (East!).
    // Projection Q is at (25.0, 35.0).
    // Q - U = (25.0 - 28.5, 35 - 35) = (-3.5, 0) [pointing West toward trail].
    // XTE = dot(Q - U, nLeft) = -3.5 * (+1) = -3.5 (trail is on user's RIGHT).
    std::cout << "--- Phase 2: Move 3.5m laterally (DEVIATING) ---" << std::endl;
    auto g2 = nav.update({28.5, 35.0}, M_PI);
    TEST_ASSERT(g2.state == NavigationState::DEVIATING, "Phase 2: State is DEVIATING");
    TEST_ASSERT(g2.guidanceMode == GuidanceMode::LATERAL_BLEND, "Phase 2: GuidanceMode is LATERAL_BLEND");
    // Verify forward reverse component (South: g.n < 0) AND correction component toward trail (West: g.e < 0)
    TEST_ASSERT(g2.guidanceVector.n < 0.0, "Phase 2: Guidance vector contains Southward forward reverse component");
    TEST_ASSERT(g2.guidanceVector.e < 0.0, "Phase 2: Guidance vector contains Westward lateral correction toward trail");

    // Phase 3: Move > 5m away (e=32.0m, > 5m lateral) for 3 updates
    std::cout << "--- Phase 3: Move > 5m away for 3 updates (OFF_TRAIL) ---" << std::endl;
    nav.update({32.0, 35.0}, M_PI);
    nav.update({32.0, 35.0}, M_PI);
    auto g3 = nav.update({32.0, 35.0}, M_PI);
    TEST_ASSERT(g3.state == NavigationState::OFF_TRAIL, "Phase 3: State is OFF_TRAIL after 3 updates");
    TEST_ASSERT(g3.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "Phase 3: GuidanceMode is DIRECT_RECOVERY");
    TEST_ASSERT(g3.recoveryActive, "Phase 3: recoveryActive is true");
    // Verify recovery vector points directly toward Q (25.0, 35.0) -> pointing West (g.e < 0, g.n ~ 0)
    TEST_ASSERT(g3.guidanceVector.e < -0.9, "Phase 3: Recovery vector points directly West toward Q");
    TEST_ASSERT(std::abs(g3.guidanceVector.n) < 0.1, "Phase 3: Pure orthogonal recovery toward trail");

    // Phase 4: Move back inside 5m (e=28.5m, XTE=3.5m <= 5m)
    std::cout << "--- Phase 4: Move back inside 5m (RECOVERING) ---" << std::endl;
    auto g4 = nav.update({28.5, 35.0}, M_PI);
    TEST_ASSERT(g4.state == NavigationState::RECOVERING, "Phase 4: State transitions to RECOVERING");
    TEST_ASSERT(g4.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "Phase 4: GuidanceMode remains DIRECT_RECOVERY");
    TEST_ASSERT(g4.recoveryActive, "Phase 4: recoveryActive is true");

    // Move to 2.2m (still > 2m) -> must STAY in RECOVERING
    auto g4b = nav.update({27.2, 35.0}, M_PI);
    TEST_ASSERT(g4b.state == NavigationState::RECOVERING, "Phase 4b: XTE=2.2m (>2m) STAYS in RECOVERING");
    TEST_ASSERT(g4b.guidanceMode == GuidanceMode::DIRECT_RECOVERY, "Phase 4b: Does NOT switch to NORMAL_CARROT yet");

    // Phase 5: Move inside 2m (e=26.5m, XTE=1.5m <= 2m)
    std::cout << "--- Phase 5: Move inside 2m (ON_TRAIL) ---" << std::endl;
    auto g5 = nav.update({26.5, 35.0}, M_PI);
    TEST_ASSERT(g5.state == NavigationState::ON_TRAIL, "Phase 5: State transitions to ON_TRAIL");
    TEST_ASSERT(g5.guidanceMode == GuidanceMode::NORMAL_CARROT, "Phase 5: GuidanceMode switches to NORMAL_CARROT");
    TEST_ASSERT(!g5.recoveryActive, "Phase 5: recoveryActive is false");

    // Phase 6: Continue along trail to P0 (0,0)
    std::cout << "--- Phase 6: Continue to P0 (RETURN_COMPLETE) ---" << std::endl;
    nav.update({25.0, 25.0}, -M_PI / 2.0); // Turning point
    nav.update({12.0, 25.0}, -M_PI / 2.0);
    nav.update({0.0, 25.0}, M_PI);         // Turning point
    nav.update({0.0, 12.0}, M_PI);
    nav.update({0.0, 2.0}, M_PI);
    nav.update({0.0, 0.5}, M_PI);
    nav.update({0.0, 0.0}, M_PI);
    auto g6 = nav.update({0.0, 0.0}, M_PI);

    TEST_ASSERT(g6.state == NavigationState::RETURN_COMPLETE, "Phase 6: State is RETURN_COMPLETE");
    TEST_ASSERT(g6.guidanceMode == GuidanceMode::ARRIVED, "Phase 6: GuidanceMode is ARRIVED");
    TEST_ASSERT(g6.guidanceVector.e == 0.0 && g6.guidanceVector.n == 0.0, "Phase 6: Zero guidance vector");
    TEST_ASSERT(g6.turnCommand == TurnCommand::STRAIGHT, "Phase 6: TurnCommand is STRAIGHT");
}

int main() {
    std::cout << "=========================================================" << std::endl;
    std::cout << "  RUNNING NATIVE M6 RECOVERY & GUIDANCE TESTS" << std::endl;
    std::cout << "=========================================================" << std::endl;

    testDeviationTransition();
    testOffTrailPersistence();
    testRecoveryThresholdHysteresis();
    testRecoveryVectorPointsTowardTrail();
    testRecoveryVectorEastOfTrail();
    testRecoveryVectorWestOfTrail();
    testLateralBlendEastSide();
    testLateralBlendWestSide();
    testGuidanceModeSelection();
    testTurnStraight();
    testTurnLeft();
    testTurnRight();
    testUTurn();
    testTurnBoundaryValues();
    testExisting180DegreeTieBreak();
    testSelfCrossingRecovery();
    testRecoveryProgressPreservation();
    testDeterministicRecoveryReplay();
    testEndToEndSyntheticScenario();

    std::cout << "\n=========================================================" << std::endl;
    std::cout << "  ALL 19 M6 NATIVE UNIT TESTS PASSED (100% PASS)" << std::endl;
    std::cout << "=========================================================" << std::endl;
    return 0;
}
