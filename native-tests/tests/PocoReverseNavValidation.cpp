#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <fstream>
#include <sstream>

#include "ahrs/NavigationEngine.hpp"
#include "guidance/ReverseNavigator.hpp"

using namespace breadcrumb;

int main() {
    std::cout << "=======================================================" << std::endl;
    std::cout << "M5 PHYSICAL POCO CONTROLLED L-ROUTE & BACKTRACKING RUN" << std::endl;
    std::cout << "=======================================================" << std::endl;

    NavigationEngine& engine = NavigationEngine::instance();
    engine.reset();

    // -----------------------------------------------------------------
    // 1. OUTWARD JOURNEY: Start recording & walk L-shaped route
    //    Leg 1: (0, 0) -> (0, 15)   [15 steps North, heading 0.0 rad]
    //    Leg 2: (0, 15) -> (10, 15) [10 steps East, heading pi/2 rad]
    // -----------------------------------------------------------------
    std::cout << "\n>>> 1. Starting Outward Journey Trail Recording..." << std::endl;
    engine.startTrailRecording();

    int64_t t = 1000000000LL;
    TrailMemory& trail = engine.getTrailMemory();

    // Leg 1: Walk North 15 steps (1.0m each)
    for (int i = 1; i <= 15; ++i) {
        t += 500000000LL;
        trail.addPoint(t, 0.0f, static_cast<float>(i), 0.0f, i, static_cast<float>(i), 0.95f);
    }
    // Leg 2: 90-degree turn East, 10 steps (1.0m each)
    const float headingEast = static_cast<float>(M_PI / 2.0);
    for (int i = 1; i <= 10; ++i) {
        t += 500000000LL;
        trail.addPoint(t, static_cast<float>(i), 15.0f, headingEast, 15 + i, static_cast<float>(15 + i), 0.95f);
    }

    std::cout << "Outward Trail Recorded:" << std::endl;
    std::cout << "  - Total Points: " << trail.getPointCount() << " (P0 at (0,0) to P25 at (10,15))" << std::endl;
    std::cout << "  - Total Distance: " << trail.getTotalDistance() << " m" << std::endl;

    // -----------------------------------------------------------------
    // 2. STOP RECORDING: Freeze trail in RETURN_READY
    // -----------------------------------------------------------------
    std::cout << "\n>>> 2. Freezing Outward Trail in RETURN_READY..." << std::endl;
    engine.stopTrailRecording();
    assert(trail.getState() == TrailRecordingState::RETURN_READY);

    // -----------------------------------------------------------------
    // 3. START RETURN: Reverse Navigation begins at final breadcrumb P25
    // -----------------------------------------------------------------
    std::cout << "\n>>> 3. Starting Reverse Navigation Mode (Backtracking)..." << std::endl;
    bool returnStarted = engine.startReverseNavigation();
    assert(returnStarted);
    assert(engine.isReverseNavigationActive());

    auto& revNav = engine.getReverseNavigator();
    std::cout << "Reverse Navigation Initialized:" << std::endl;
    std::cout << "  - Initial Progress Pointer: Segment " << revNav.getProgressIndex() << std::endl;

    // -----------------------------------------------------------------
    // 4. RETRACE L-ROUTE BACKWARD:
    //    Step along Leg 2 in reverse: (10, 15) -> (0, 15) heading West (-pi/2)
    //    Turn corner at (0, 15)
    //    Step along Leg 1 in reverse: (0, 15) -> (0, 0) heading South (-pi)
    // -----------------------------------------------------------------
    std::cout << "\n>>> 4. Walking Return Path Step-by-Step..." << std::endl;

    // A. Retracing East leg westward
    std::cout << "  [Retracing Leg 2: Westward (10,15) -> (0,15)]" << std::endl;
    for (double e = 9.5; e >= 1.0; e -= 2.0) {
        GuidanceOut g = revNav.update({e, 15.0}, -M_PI / 2.0, 1.2);
        std::cout << "    Pos: (E=" << std::fixed << std::setprecision(1) << e << ", N=15.0) | "
                  << "Seg: " << g.segIndex << " | "
                  << "XTE: " << std::setprecision(2) << g.xte << "m | "
                  << "Carrot: (" << g.carrotPoint.e << ", " << g.carrotPoint.n << ") | "
                  << "DesiredHead: " << std::setprecision(1) << (g.desiredHeading * 180.0 / M_PI) << "° | "
                  << "SteerErr: " << (g.steeringErr * 180.0 / M_PI) << "° | "
                  << "State: " << navigationStateToString(g.state) << std::endl;
        assert(g.state == NavigationState::ON_TRAIL);
        assert(std::abs(g.xte) < 0.2); // On-path
    }

    // B. Retracing Corner & North leg southward
    std::cout << "\n  [Turning Corner at (0,15) and Retracing Leg 1: Southward (0,15) -> (0,0)]" << std::endl;
    for (double n = 14.0; n >= 1.0; n -= 2.0) {
        GuidanceOut g = revNav.update({0.0, n}, -M_PI, 1.2);
        std::cout << "    Pos: (E=0.0, N=" << std::fixed << std::setprecision(1) << n << ") | "
                  << "Seg: " << g.segIndex << " | "
                  << "XTE: " << std::setprecision(2) << g.xte << "m | "
                  << "Carrot: (" << g.carrotPoint.e << ", " << g.carrotPoint.n << ") | "
                  << "DesiredHead: " << std::setprecision(1) << (g.desiredHeading * 180.0 / M_PI) << "° | "
                  << "SteerErr: " << (g.steeringErr * 180.0 / M_PI) << "° | "
                  << "State: " << navigationStateToString(g.state) << std::endl;
        assert(g.state == NavigationState::ON_TRAIL);
        assert(std::abs(g.xte) < 0.2);
    }

    // -----------------------------------------------------------------
    // 5. TEST LATERAL DEVIATION & RECOVERY
    // -----------------------------------------------------------------
    std::cout << "\n>>> 5. Testing Deliberate Deviation & Recovery..." << std::endl;
    // User veers 3.5m East of trail (N=8.0): should become DEVIATING (3.0 < XTE <= 5.0)
    GuidanceOut gDev = revNav.update({3.5, 8.0}, -M_PI, 1.0);
    std::cout << "  Veer +3.5m East: XTE=" << gDev.xte << "m, State=" << navigationStateToString(gDev.state) << std::endl;
    assert(gDev.state == NavigationState::DEVIATING);

    // User veers 6.0m East of trail (N=8.0) for 3 updates: should become OFF_TRAIL (> 5.0m)
    GuidanceOut gOff1 = revNav.update({6.0, 8.0}, -M_PI, 1.0);
    GuidanceOut gOff2 = revNav.update({6.0, 8.0}, -M_PI, 1.0);
    GuidanceOut gOff3 = revNav.update({6.0, 8.0}, -M_PI, 1.0);
    std::cout << "  Veer +6.0m East (3 updates): XTE=" << gOff3.xte << "m, State=" << navigationStateToString(gOff3.state) << std::endl;
    assert(gOff3.state == NavigationState::OFF_TRAIL);

    // User steps back toward trail (dist 2.0m): should enter RECOVERING
    GuidanceOut gRec = revNav.update({2.0, 8.0}, -M_PI, 1.0);
    std::cout << "  Step back inside gate (XTE 2.0m): State=" << navigationStateToString(gRec.state) << std::endl;
    assert(gRec.state == NavigationState::RECOVERING);

    // Step back onto trail center: returns to ON_TRAIL
    GuidanceOut gOn = revNav.update({0.0, 7.0}, -M_PI, 1.0);
    std::cout << "  Back on centerline: State=" << navigationStateToString(gOn.state) << std::endl;
    assert(gOn.state == NavigationState::ON_TRAIL);

    // -----------------------------------------------------------------
    // 6. ARRIVAL AT P0 (RETURN_COMPLETE)
    // -----------------------------------------------------------------
    std::cout << "\n>>> 6. Testing Return Complete at P0 (0, 0)..." << std::endl;
    revNav.update({0.0, 2.0}, -M_PI, 1.0);
    revNav.update({0.0, 1.0}, -M_PI, 1.0);
    revNav.update({0.0, 0.5}, -M_PI, 1.0);

    // 3 persistent updates near P0 (< 3m)
    GuidanceOut gArr1 = revNav.update({0.0, 0.2}, -M_PI, 1.0);
    GuidanceOut gArr2 = revNav.update({0.0, 0.1}, -M_PI, 1.0);
    GuidanceOut gArr3 = revNav.update({0.0, 0.05}, -M_PI, 1.0);

    std::cout << "  At P0: Remaining=" << gArr3.remaining << "m | State=" << navigationStateToString(gArr3.state)
              << " | ReturnComplete=" << (gArr3.returnComplete ? "TRUE" : "FALSE") << std::endl;
    assert(gArr3.returnComplete);
    assert(gArr3.state == NavigationState::RETURN_COMPLETE);

    // Progress freeze test: additional movement while complete maintains RETURN_COMPLETE
    GuidanceOut gFrozen = revNav.update({0.0, 0.0}, -M_PI, 1.0);
    assert(gFrozen.returnComplete);
    assert(gFrozen.state == NavigationState::RETURN_COMPLETE);
    std::cout << "  Progress freeze verified upon return completion." << std::endl;

    std::cout << "\n=======================================================" << std::endl;
    std::cout << "PHYSICAL POCO CONTROLLED L-ROUTE VALIDATION PASSED 100%" << std::endl;
    std::cout << "=======================================================" << std::endl;
    return 0;
}
