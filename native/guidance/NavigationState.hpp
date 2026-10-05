#pragma once

#include <cstdint>
#include <cmath>

namespace breadcrumb {

/**
 * Navigation state machine per gpt_technical_final.md §18.5 and M6 specification.
 */
enum class NavigationState : uint32_t {
    ON_TRAIL = 0,        // Valid gated trail candidate, |XTE| <= 3.0m
    DEVIATING = 1,       // 3.0m < |XTE| <= 5.0m
    OFF_TRAIL = 2,       // Sustained |XTE| > 5.0m or sustained gating failure (>= 3 updates)
    RECOVERING = 3,      // Candidate reacquired (|XTE| <= 5.0m), persists until |XTE| <= 2.0m
    RETURN_COMPLETE = 4  // Progress reached beginning (P0) within arrival radius
};

inline const char* navigationStateToString(NavigationState state) {
    switch (state) {
        case NavigationState::ON_TRAIL: return "ON_TRAIL";
        case NavigationState::DEVIATING: return "DEVIATING";
        case NavigationState::OFF_TRAIL: return "OFF_TRAIL";
        case NavigationState::RECOVERING: return "RECOVERING";
        case NavigationState::RETURN_COMPLETE: return "RETURN_COMPLETE";
        default: return "UNKNOWN";
    }
}

/**
 * M6 Explicit Guidance Modes (§18.1-18.4, M6 Spec).
 */
enum class GuidanceMode : uint32_t {
    NORMAL_CARROT = 0,   // Target = look-ahead carrot C on reverse trail
    LATERAL_BLEND = 1,   // Target = reverse trail tangent with lateral correction
    DIRECT_RECOVERY = 2, // Target = projected point Q on recovery segment
    ARRIVED = 3          // At destination P0, zero guidance vector
};

inline const char* guidanceModeToString(GuidanceMode mode) {
    switch (mode) {
        case GuidanceMode::NORMAL_CARROT: return "NORMAL_CARROT";
        case GuidanceMode::LATERAL_BLEND: return "LATERAL_BLEND";
        case GuidanceMode::DIRECT_RECOVERY: return "DIRECT_RECOVERY";
        case GuidanceMode::ARRIVED: return "ARRIVED";
        default: return "UNKNOWN";
    }
}

/**
 * M6 Discrete Turn Commands (§18.4, M6 Spec).
 * Classified from steering error DeltaPsi = wrapPi(desiredHeading - userHeading).
 */
enum class TurnCommand : uint32_t {
    STRAIGHT = 0,   // |DeltaPsi| < 15 deg
    TURN_LEFT = 1,  // -165 deg <= DeltaPsi <= -15 deg
    TURN_RIGHT = 2, // +15 deg <= DeltaPsi <= +165 deg
    U_TURN = 3      // |DeltaPsi| > 165 deg
};

inline const char* turnCommandToString(TurnCommand cmd) {
    switch (cmd) {
        case TurnCommand::STRAIGHT: return "STRAIGHT";
        case TurnCommand::TURN_LEFT: return "TURN_LEFT";
        case TurnCommand::TURN_RIGHT: return "TURN_RIGHT";
        case TurnCommand::U_TURN: return "U_TURN";
        default: return "UNKNOWN";
    }
}

/**
 * Classifies resolved steering error (in radians) into TurnCommand.
 */
inline TurnCommand classifyTurnCommand(double steeringErrRad) {
    constexpr double DEG2RAD = M_PI / 180.0;
    constexpr double EPS = 1e-6;
    const double deg = steeringErrRad / DEG2RAD;
    const double absDeg = std::abs(deg);
    if (absDeg < 15.0 - EPS) {
        return TurnCommand::STRAIGHT;
    }
    if (absDeg > 165.0 + EPS) {
        return TurnCommand::U_TURN;
    }
    if (deg >= 15.0 - EPS && deg <= 165.0 + EPS) {
        return TurnCommand::TURN_RIGHT;
    }
    return TurnCommand::TURN_LEFT;
}

} // namespace breadcrumb

