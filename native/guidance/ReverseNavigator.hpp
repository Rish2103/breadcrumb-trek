#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include "../trail/TrailPoint.hpp"
#include "../trail/TrailMemory.hpp"
#include "NavigationState.hpp"

namespace breadcrumb {

struct P2 {
    double e, n;
};

inline P2 operator-(P2 a, P2 b) { return {a.e - b.e, a.n - b.n}; }
inline P2 operator+(P2 a, P2 b) { return {a.e + b.e, a.n + b.n}; }
inline P2 operator*(P2 a, double s) { return {a.e * s, a.n * s}; }
inline double dot(P2 a, P2 b) { return a.e * b.e + a.n * b.n; }
inline double norm(P2 a) { return std::sqrt(dot(a, a)); }
inline double bearing(P2 v) { return std::atan2(v.e, v.n); }
inline double wrapPi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

/**
 * Output of the reverse navigation and backtracking matching engine.
 */
struct GuidanceOut {
    // Cross-track error (m); sign convention: + = trail is to user's LEFT, - = trail is to user's RIGHT
    double xte = 0.0;
    double absXte = 0.0;

    // Steering error (rad wrapped [-pi, +pi]); + = turn RIGHT (clockwise), - = turn LEFT (counter-clockwise)
    double steeringErr = 0.0;
    double desiredHeading = 0.0; // Desired bearing to carrot (rad, clockwise from North)

    double remaining = 0.0;      // Remaining reverse path distance to P0 (m)
    int segIndex = 0;           // Matched reverse segment index (0 = Pn -> P(n-1), ..., N-2 = P1 -> P0)
    int reverseProgressIndex = 0; // Current reverse progress pointer (segment index)

    P2 projPoint{0.0, 0.0};     // Orthogonal projection Q on matched segment
    P2 carrotPoint{0.0, 0.0};   // Look-ahead carrot C on reverse trail

    NavigationState state = NavigationState::ON_TRAIL;
    GuidanceMode guidanceMode = GuidanceMode::NORMAL_CARROT;
    TurnCommand turnCommand = TurnCommand::STRAIGHT;
    bool recoveryActive = false;
    P2 recoveryTarget{0.0, 0.0};
    P2 guidanceVector{0.0, 0.0};
    bool offTrail = false;
    bool returnComplete = false;

    // Matching-cost diagnostics (§17.4)
    double jDist = 0.0;         // lambda_d * d_hat
    double jSeq = 0.0;          // lambda_s * E_seq
    double jHead = 0.0;         // lambda_h * E_head
    double jConf = 0.0;         // lambda_c * E_conf
    double jTotal = 0.0;        // Sum J
};

/**
 * Deterministic reverse navigation and backtracking matching engine.
 *
 * Implements gpt_technical_final.md §17 (Reverse navigation) and §18 (Closed-loop guidance).
 *
 * Provides a reverse view over TrailMemory:
 *   reverseIndex 0     -> Pn (destination at journey end)
 *   reverseIndex 1     -> P(n-1)
 *   ...
 *   reverseIndex n     -> P0 (origin at journey start)
 *
 * Reverse segment i connects A = P(n - i) to B = P(n - i - 1).
 */
class ReverseNavigator {
public:
    struct Cfg {
        int backWin = 2;             // Bounded search window backwards (segments)
        int fwdWin = 12;            // Bounded search window forwards (segments)
        double lamD = 1.0;          // Distance cost weight
        double lamS = 0.3;          // Sequence difference cost weight
        double lamH = 0.4;          // Heading error cost weight
        double lamC = 0.3;          // Confidence cost weight
        double d0 = 1.0;            // Distance normalization offset (m)
        double dMin = 5.0;          // Minimum gating distance (m) -> OFF_TRAIL threshold
        double kGate = 3.0;         // Gating factor scaling with sigmaPos
        double dJswitch = 0.15;     // Hysteresis threshold to switch segments
        double lookAheadT = 2.0;    // Look-ahead time (s)
        double dLmin = 2.0;         // Minimum look-ahead distance (m)
        double ke = 0.25;           // Lateral blend gain (1/m)
        double keCap = 2.0;         // Maximum lateral blend cap
        bool useBlend = false;      // Optional override
        double piEps = 0.05;        // 180° tie-break band (rad ~2.86 deg)
        double dDev = 3.0;          // Deviation state threshold (m)
        double dRecov = 2.0;        // Recovery threshold to return to ON_TRAIL (m)
        double rArrive = 3.0;       // Return complete arrival radius (m)
        int persistUpdates = 3;     // Debounce persistence updates for state changes
        double sigmaPos = 0.0;      // Deterministic MVP position uncertainty fallback
    };

    ReverseNavigator();

    void setConfig(const Cfg& cfg) { cfg_ = cfg; }
    const Cfg& getConfig() const { return cfg_; }

    /**
     * Initializes reverse navigation on the frozen trail.
     * Sets progress pointer to reverseIndex 0 (segment 0: Pn -> P(n-1)).
     */
    bool startReturn(const TrailMemory& trail);

    /**
     * Stops reverse navigation and resets state.
     */
    void stopReturn();

    bool isActive() const { return active_; }

    /**
     * Accesses trail point in reverse order without physical vector duplication.
     * reverseIndex 0 -> Pn, reverseIndex (N-1) -> P0.
     */
    bool getReversePoint(size_t reverseIndex, TrailPoint& outPoint) const;

    size_t getPointCount() const { return trailPoints_.size(); }
    int getProgressIndex() const { return curSeg_; }

    /**
     * Executes deterministic backtracking matching update step.
     *
     * @param pu Current user position in local ENU frame (meters East, North)
     * @param psiU Current user heading (radians, clockwise from North [-pi, +pi])
     * @param speed Estimated walking speed (m/s)
     * @return GuidanceOut structure containing all matching, carrot, and steering outputs
     */
    GuidanceOut update(P2 pu, double psiU, double speed = 1.2);

    /**
     * Static helper: 2D point-to-segment projection.
     */
    static void projectOnSegment(P2 p, P2 a, P2 b, P2& outQ, double& outU, double& outDist);

    /**
     * Static helper: calculates reverse segment heading pointing from A to B.
     */
    static double computeSegmentBearing(P2 a, P2 b);

    const GuidanceOut& getLatestGuidance() const { return latestGuidance_; }

private:
    Cfg cfg_;
    bool active_ = false;

    // Snapshot of forward trail points referenced during return
    std::vector<TrailPoint> trailPoints_;
    // Precomputed cumulative arc lengths along reverse polyline
    std::vector<double> reverseCumDist_;

    int curSeg_ = 0;
    int prevTurn_ = +1; // +1 = RIGHT, -1 = LEFT for 180° tie-break

    // State machine debounce counters
    int offTrailCounter_ = 0;
    int onTrailCounter_ = 0;
    int arriveCounter_ = 0;
    NavigationState navState_ = NavigationState::ON_TRAIL;
    GuidanceOut latestGuidance_{};
};

} // namespace breadcrumb
