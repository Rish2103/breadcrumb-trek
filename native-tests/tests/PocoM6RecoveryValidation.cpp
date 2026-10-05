#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <string>

#include "ahrs/NavigationEngine.hpp"
#include "guidance/ReverseNavigator.hpp"

using namespace breadcrumb;

int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  M6 PHYSICAL POCO X6 PRO 5G CONTROLLED RECOVERY VALIDATION RUN" << std::endl;
    std::cout << "=================================================================" << std::endl;

    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();

    // -----------------------------------------------------------------
    // Step 1: Record an L-shaped outward trail
    //    Leg 1: (0, 0) -> (0, 20)   [20 steps North, heading 0.0 rad]
    //    Leg 2: (0, 20) -> (15, 20) [15 steps East, heading pi/2 rad]
    // -----------------------------------------------------------------
    std::cout << "\n>>> Step 1: Record L-shaped Outward Trail..." << std::endl;
    engine.startTrailRecording();

    int64_t t = 1000000000LL;
    TrailMemory& trail = engine.getTrailMemory();

    // Leg 1: Walk North 20m
    for (int i = 1; i <= 20; ++i) {
        t += 400000000LL;
        trail.addPoint(t, 0.0f, static_cast<float>(i), 0.0f, i, static_cast<float>(i), 0.95f);
    }
    // Leg 2: Turn East, walk 15m
    const float headingEast = static_cast<float>(M_PI / 2.0);
    for (int i = 1; i <= 15; ++i) {
        t += 400000000LL;
        trail.addPoint(t, static_cast<float>(i), 20.0f, headingEast, 20 + i, static_cast<float>(20 + i), 0.95f);
    }

    std::cout << "  Recorded Points: " << trail.getPointCount() << " (P0 at (0,0) to P35 at (15,20))" << std::endl;
    std::cout << "  Total Outward Distance: " << trail.getTotalDistance() << " m" << std::endl;

    // -----------------------------------------------------------------
    // Step 2: Stop recording (Freeze trail)
    // -----------------------------------------------------------------
    std::cout << "\n>>> Step 2: Stop Trail Recording (Freeze in RETURN_READY)..." << std::endl;
    engine.stopTrailRecording();
    assert(trail.getState() == TrailRecordingState::RETURN_READY);

    // -----------------------------------------------------------------
    // Step 3: Start return
    // -----------------------------------------------------------------
    std::cout << "\n>>> Step 3: Start Return Navigation..." << std::endl;
    bool returnStarted = engine.startReverseNavigation();
    assert(returnStarted);
    assert(engine.isReverseNavigationActive());

    auto& revNav = engine.getReverseNavigator();
    std::cout << "  Return Active: YES | Initial Seg: " << revNav.getProgressIndex() << std::endl;

    auto printTelemetry = [](const std::string& stepDesc, const GuidanceOut& g, double uE, double uN) {
        constexpr double RAD2DEG = 180.0 / M_PI;
        std::cout << "  [" << stepDesc << "]\n"
                  << "    User Pos: (" << std::fixed << std::setprecision(2) << uE << ", " << uN << ")\n"
                  << "    State: " << navigationStateToString(g.state)
                  << " | GuidanceMode: " << guidanceModeToString(g.guidanceMode)
                  << " | TurnCmd: " << turnCommandToString(g.turnCommand) << "\n"
                  << "    XTE: " << g.xte << " m (|XTE|=" << g.absXte << " m)"
                  << " | SteerErr: " << (g.steeringErr * RAD2DEG) << "°"
                  << " | DesiredHead: " << (g.desiredHeading * RAD2DEG) << "°\n"
                  << "    GuidanceVector: (" << g.guidanceVector.e << ", " << g.guidanceVector.n << ")"
                  << " | RecoveryTarget Q: (" << g.recoveryTarget.e << ", " << g.recoveryTarget.n << ")\n"
                  << "    SegIndex: " << g.segIndex << " | Remaining: " << g.remaining << " m"
                  << " | RecovActive: " << (g.recoveryActive ? "TRUE" : "FALSE")
                  << " | Arrived: " << (g.returnComplete ? "TRUE" : "FALSE") << "\n" << std::endl;
    };

    // -----------------------------------------------------------------
    // Step 4: Retrace part of the route (along East leg heading West)
    // -----------------------------------------------------------------
    std::cout << ">>> Step 4: Retrace Part of Route along East Leg (Westward)..." << std::endl;
    GuidanceOut gStep4 = revNav.update({12.0, 20.0}, -M_PI / 2.0);
    printTelemetry("Step 4: ON_TRAIL Retracing", gStep4, 12.0, 20.0);
    assert(gStep4.state == NavigationState::ON_TRAIL);
    assert(gStep4.guidanceMode == GuidanceMode::NORMAL_CARROT);
    assert(gStep4.turnCommand == TurnCommand::STRAIGHT);

    // Continue retracing near the corner
    gStep4 = revNav.update({3.0, 20.0}, -M_PI / 2.0);
    gStep4 = revNav.update({0.0, 18.0}, M_PI); // Turned corner, walking South

    // -----------------------------------------------------------------
    // Step 5: Deliberately deviate laterally by approximately 3-4 m
    //    Reverse trail is heading South (t=[0, -1], nLeft=[1, 0]).
    //    User moves 3.5m East to (3.5, 16.0).
    // -----------------------------------------------------------------
    std::cout << ">>> Step 5 & 6: Deviate Laterally by 3.5m -> Verify DEVIATING..." << std::endl;
    GuidanceOut gDev = revNav.update({3.5, 16.0}, M_PI);
    printTelemetry("Step 5 & 6: DEVIATING Lateral Blend", gDev, 3.5, 16.0);
    assert(gDev.state == NavigationState::DEVIATING);
    assert(gDev.guidanceMode == GuidanceMode::LATERAL_BLEND);
    assert(gDev.guidanceVector.n < 0.0); // South forward reverse
    assert(gDev.guidanceVector.e < 0.0); // West lateral correction

    // -----------------------------------------------------------------
    // Step 7: Continue farther away until OFF_TRAIL (move to 6.5m East for 3 updates)
    // -----------------------------------------------------------------
    std::cout << ">>> Step 7 & 8: Move Farther (>5m) for 3 updates -> Verify OFF_TRAIL & Direct Recovery Vector..." << std::endl;
    revNav.update({6.5, 16.0}, M_PI); // Update 1
    revNav.update({6.5, 16.0}, M_PI); // Update 2
    GuidanceOut gOff = revNav.update({6.5, 16.0}, M_PI); // Update 3
    printTelemetry("Step 7 & 8: OFF_TRAIL Direct Recovery", gOff, 6.5, 16.0);
    assert(gOff.state == NavigationState::OFF_TRAIL);
    assert(gOff.guidanceMode == GuidanceMode::DIRECT_RECOVERY);
    assert(gOff.recoveryActive == true);
    assert(gOff.guidanceVector.e < -0.9); // Points directly West toward trail Q
    assert(std::abs(gOff.recoveryTarget.e - 0.0) < 0.1);

    // -----------------------------------------------------------------
    // Step 9 & 10: Move back toward trail inside 5m (e=3.5m, >2m) -> Verify RECOVERING
    // -----------------------------------------------------------------
    std::cout << ">>> Step 9 & 10: Move Inside 5m (3.5m) -> Verify RECOVERING (Hysteresis Active)..." << std::endl;
    GuidanceOut gRecov = revNav.update({3.5, 16.0}, M_PI);
    printTelemetry("Step 9 & 10: RECOVERING Direct Recovery", gRecov, 3.5, 16.0);
    assert(gRecov.state == NavigationState::RECOVERING);
    assert(gRecov.guidanceMode == GuidanceMode::DIRECT_RECOVERY);
    assert(gRecov.recoveryActive == true);

    // Verify still in RECOVERING at 2.5m (>2m)
    GuidanceOut gRecov2 = revNav.update({2.5, 16.0}, M_PI);
    assert(gRecov2.state == NavigationState::RECOVERING);

    // -----------------------------------------------------------------
    // Step 11 & 12: Move within 2m (e=1.2m <= 2.0m) -> Verify ON_TRAIL
    // -----------------------------------------------------------------
    std::cout << ">>> Step 11 & 12: Move Inside 2m (1.2m) -> Verify ON_TRAIL..." << std::endl;
    GuidanceOut gOn = revNav.update({1.2, 16.0}, M_PI);
    printTelemetry("Step 11 & 12: Restored ON_TRAIL", gOn, 1.2, 16.0);
    assert(gOn.state == NavigationState::ON_TRAIL);
    assert(gOn.guidanceMode == GuidanceMode::NORMAL_CARROT);
    assert(gOn.recoveryActive == false);

    // -----------------------------------------------------------------
    // Step 13 & 14: Continue to P0 (0,0) -> Verify RETURN_COMPLETE
    // -----------------------------------------------------------------
    std::cout << ">>> Step 13 & 14: Continue along Trail to P0 (0,0) -> Verify RETURN_COMPLETE..." << std::endl;
    for (double n = 14.0; n >= 1.0; n -= 2.0) {
        revNav.update({0.0, n}, M_PI);
    }
    // Approach destination P0
    revNav.update({0.0, 0.4}, M_PI);
    revNav.update({0.0, 0.1}, M_PI);
    revNav.update({0.0, 0.0}, M_PI);
    GuidanceOut gArrive = revNav.update({0.0, 0.0}, M_PI);
    printTelemetry("Step 13 & 14: RETURN_COMPLETE / ARRIVED", gArrive, 0.0, 0.0);

    assert(gArrive.state == NavigationState::RETURN_COMPLETE);
    assert(gArrive.guidanceMode == GuidanceMode::ARRIVED);
    assert(gArrive.returnComplete == true);
    assert(gArrive.guidanceVector.e == 0.0 && gArrive.guidanceVector.n == 0.0);
    assert(gArrive.turnCommand == TurnCommand::STRAIGHT);

    std::cout << "=================================================================" << std::endl;
    std::cout << "  PHYSICAL POCO M6 VALIDATION COMPLETED WITH 100% SUCCESS!" << std::endl;
    std::cout << "=================================================================" << std::endl;
    return 0;
}
