#include "ReverseNavigator.hpp"

namespace breadcrumb {

ReverseNavigator::ReverseNavigator() = default;

bool ReverseNavigator::startReturn(const TrailMemory& trail) {
    if (trail.getPointCount() < 2) {
        return false;
    }

    trailPoints_ = trail.getPoints();
    const size_t N = trailPoints_.size();

    // Precompute cumulative arc length along reverse polyline:
    // R[0] = Pn (origin of return), R[N-1] = P0 (destination of return)
    reverseCumDist_.assign(N, 0.0);
    for (size_t i = 1; i < N; ++i) {
        const TrailPoint& pPrev = trailPoints_[N - i];     // R[i-1]
        const TrailPoint& pCurr = trailPoints_[N - 1 - i]; // R[i]
        const double de = pCurr.east - pPrev.east;
        const double dn = pCurr.north - pPrev.north;
        reverseCumDist_[i] = reverseCumDist_[i - 1] + std::sqrt(de * de + dn * dn);
    }

    curSeg_ = 0;
    prevTurn_ = +1; // Default clockwise / right for 180° tie-break
    offTrailCounter_ = 0;
    onTrailCounter_ = 0;
    arriveCounter_ = 0;
    navState_ = NavigationState::ON_TRAIL;
    active_ = true;

    latestGuidance_ = GuidanceOut{};
    latestGuidance_.state = NavigationState::ON_TRAIL;
    return true;
}

void ReverseNavigator::stopReturn() {
    active_ = false;
    trailPoints_.clear();
    reverseCumDist_.clear();
    curSeg_ = 0;
    offTrailCounter_ = 0;
    onTrailCounter_ = 0;
    arriveCounter_ = 0;
    navState_ = NavigationState::ON_TRAIL;
    latestGuidance_ = GuidanceOut{};
}

bool ReverseNavigator::getReversePoint(size_t reverseIndex, TrailPoint& outPoint) const {
    if (reverseIndex >= trailPoints_.size()) {
        return false;
    }
    outPoint = trailPoints_[trailPoints_.size() - 1 - reverseIndex];
    return true;
}

void ReverseNavigator::projectOnSegment(P2 p, P2 a, P2 b, P2& outQ, double& outU, double& outDist) {
    const P2 ab = b - a;
    const double L2 = dot(ab, ab);
    if (L2 < 1e-9) {
        outQ = a;
        outU = 0.0;
        outDist = norm(p - a);
        return;
    }
    outU = std::clamp(dot(p - a, ab) / L2, 0.0, 1.0);
    outQ = a + ab * outU;
    outDist = norm(p - outQ);
}

double ReverseNavigator::computeSegmentBearing(P2 a, P2 b) {
    const P2 ab = b - a;
    return std::atan2(ab.e, ab.n);
}

GuidanceOut ReverseNavigator::update(P2 pu, double psiU, double speed) {
    GuidanceOut out{};
    const size_t N = trailPoints_.size();
    if (!active_ || N < 2) {
        out.offTrail = true;
        out.state = NavigationState::OFF_TRAIL;
        out.guidanceMode = GuidanceMode::DIRECT_RECOVERY;
        out.turnCommand = TurnCommand::STRAIGHT;
        out.recoveryActive = true;
        out.recoveryTarget = {0.0, 0.0};
        out.guidanceVector = {0.0, 0.0};
        latestGuidance_ = out;
        return out;
    }

    const int totalSegs = static_cast<int>(N - 1);
    const double gate = std::max(cfg_.dMin, cfg_.kGate * cfg_.sigmaPos);

    // If RETURN_COMPLETE has already been reached, freeze progress and return completion state
    if (navState_ == NavigationState::RETURN_COMPLETE) {
        const TrailPoint& p0 = trailPoints_[0]; // P0
        const P2 dest{p0.east, p0.north};
        out.returnComplete = true;
        out.state = NavigationState::RETURN_COMPLETE;
        out.guidanceMode = GuidanceMode::ARRIVED;
        out.turnCommand = TurnCommand::STRAIGHT;
        out.recoveryActive = false;
        out.recoveryTarget = dest;
        out.guidanceVector = {0.0, 0.0};
        out.segIndex = totalSegs - 1;
        out.reverseProgressIndex = totalSegs - 1;
        out.projPoint = dest;
        out.carrotPoint = dest;
        out.remaining = 0.0;
        out.xte = 0.0;
        out.absXte = norm(pu - dest);
        out.desiredHeading = psiU;
        out.steeringErr = 0.0;
        latestGuidance_ = out;
        return out;
    }

    // 1. Search Windowed Matching (§17.3–§17.4)
    int best = curSeg_;
    double bestJ = 1e18;
    double curJ = 1e18;
    bool anyGated = false;

    // Diagnostic components for best candidate
    double bestJDist = 0.0, bestJSeq = 0.0, bestJHead = 0.0, bestJConf = 0.0;
    double curJDist = 0.0, curJSeq = 0.0, curJHead = 0.0, curJConf = 0.0;

    const int lo = std::max(0, curSeg_ - cfg_.backWin);
    const int hi = std::min(totalSegs - 1, curSeg_ + cfg_.fwdWin);

    for (int i = lo; i <= hi; ++i) {
        // Reverse segment i: from R[i] to R[i+1]
        // R[i] = trailPoints_[N - 1 - i], R[i+1] = trailPoints_[N - 1 - (i + 1)]
        const TrailPoint& ptA = trailPoints_[N - 1 - i];
        const TrailPoint& ptB = trailPoints_[N - 1 - (i + 1)];
        const P2 a{ptA.east, ptA.north};
        const P2 b{ptB.east, ptB.north};

        P2 q;
        double u, d;
        projectOnSegment(pu, a, b, q, u, d);

        // Gating (§17.4): reject candidate if d > d_gate unless candidate is currently active segment
        if (d > gate && i != curSeg_) {
            continue;
        }
        if (d <= gate) {
            anyGated = true;
        }

        const double dHat = d / (cfg_.sigmaPos + cfg_.d0);
        const double Eseq = static_cast<double>(std::abs(i - curSeg_)) / std::max(totalSegs - 1, 1);
        const double segHeading = computeSegmentBearing(a, b);
        const double Ehead = std::abs(wrapPi(psiU - segHeading)) / M_PI;
        const double Econf = 1.0 - static_cast<double>(ptA.cMag);

        const double termDist = cfg_.lamD * dHat;
        const double termSeq = cfg_.lamS * Eseq;
        const double termHead = cfg_.lamH * Ehead;
        const double termConf = cfg_.lamC * Econf;
        const double J = termDist + termSeq + termHead + termConf;

        if (i == curSeg_) {
            curJ = J;
            curJDist = termDist;
            curJSeq = termSeq;
            curJHead = termHead;
            curJConf = termConf;
        }

        if (J < bestJ) {
            bestJ = J;
            best = i;
            bestJDist = termDist;
            bestJSeq = termSeq;
            bestJHead = termHead;
            bestJConf = termConf;
        }
    }

    // 2. Hysteresis Check (§17.4)
    if (best != curSeg_ && bestJ < curJ - cfg_.dJswitch) {
        curSeg_ = best;
        out.jDist = bestJDist;
        out.jSeq = bestJSeq;
        out.jHead = bestJHead;
        out.jConf = bestJConf;
        out.jTotal = bestJ;
    } else {
        out.jDist = curJDist;
        out.jSeq = curJSeq;
        out.jHead = curJHead;
        out.jConf = curJConf;
        out.jTotal = curJ;
    }

    // 3. Projection on selected active reverse segment
    const TrailPoint& activePtA = trailPoints_[N - 1 - curSeg_];
    const TrailPoint& activePtB = trailPoints_[N - 1 - (curSeg_ + 1)];
    const P2 a{activePtA.east, activePtA.north};
    const P2 b{activePtB.east, activePtB.north};

    P2 q;
    double u, d;
    projectOnSegment(pu, a, b, q, u, d);

    const P2 ab = b - a;
    const double segL = std::max(norm(ab), 1e-6);
    const P2 t = ab * (1.0 / segL);       // Reverse tangent pointing towards P0
    const P2 nLeft{-t.n, t.e};            // Left normal (pointing 90° counter-clockwise from tangent)
    const P2 e = q - pu;                  // Vector from user to projected point on trail

    out.xte = dot(e, nLeft);              // Signed XTE (+ = trail is to user's LEFT)
    out.absXte = d;
    out.segIndex = curSeg_;
    out.reverseProgressIndex = curSeg_;
    out.projPoint = q;

    // Remaining distance along reverse polyline to P0
    const double remainingFromSegEnd = reverseCumDist_[N - 1] - reverseCumDist_[curSeg_ + 1];
    const double remainingOnCurrentSeg = (1.0 - u) * segL;
    out.remaining = std::max(0.0, remainingFromSegEnd + remainingOnCurrentSeg);

    // 4. Adaptive Look-Ahead Carrot (§17.5 / §18.2)
    double remLookAhead = std::max(cfg_.dLmin, speed * cfg_.lookAheadT);
    remLookAhead = std::min(remLookAhead, out.remaining); // Never advance beyond P0

    P2 c = q;
    int s = curSeg_;
    while (true) {
        if (s >= totalSegs) {
            c = {trailPoints_[0].east, trailPoints_[0].north}; // P0
            break;
        }
        const TrailPoint& ptNext = trailPoints_[N - 1 - (s + 1)];
        const P2 segEnd{ptNext.east, ptNext.north};
        const double distToEnd = norm(segEnd - c);

        if (remLookAhead <= distToEnd || s >= totalSegs - 1) {
            const P2 dir = (distToEnd > 1e-9) ? (segEnd - c) * (1.0 / distToEnd) : t;
            c = c + dir * std::min(remLookAhead, distToEnd);
            break;
        }
        remLookAhead -= distToEnd;
        c = segEnd;
        ++s;
    }
    out.carrotPoint = c;

    // 5. Navigation State Machine & Arrival Detection (§18.5–§18.6, M6 Spec)
    const TrailPoint& p0 = trailPoints_[0];
    const double distToP0 = norm(pu - P2{p0.east, p0.north});

    if (curSeg_ >= totalSegs - 1 && out.remaining <= cfg_.rArrive && distToP0 <= cfg_.rArrive) {
        ++arriveCounter_;
        if (arriveCounter_ >= cfg_.persistUpdates) {
            navState_ = NavigationState::RETURN_COMPLETE;
            out.returnComplete = true;
        }
    } else {
        arriveCounter_ = 0;
    }

    if (navState_ != NavigationState::RETURN_COMPLETE) {
        if (!anyGated || d > gate) {
            // Outside gating (|XTE| > 5.0m or gating rejected)
            ++offTrailCounter_;
            onTrailCounter_ = 0;
            if (offTrailCounter_ >= cfg_.persistUpdates) {
                navState_ = NavigationState::OFF_TRAIL;
            } else if (navState_ != NavigationState::OFF_TRAIL && navState_ != NavigationState::RECOVERING) {
                // While debouncing towards OFF_TRAIL, user is DEVIATING
                navState_ = NavigationState::DEVIATING;
            }
        } else {
            // Inside gating (|XTE| <= 5.0m)
            offTrailCounter_ = 0;
            ++onTrailCounter_;
            if (navState_ == NavigationState::OFF_TRAIL) {
                // OFF_TRAIL -> RECOVERING when candidate reacquired (|XTE| <= 5.0m)
                navState_ = NavigationState::RECOVERING;
                if (d <= cfg_.dRecov) {
                    navState_ = NavigationState::ON_TRAIL;
                }
            } else if (navState_ == NavigationState::RECOVERING) {
                // Hysteresis: Stays in RECOVERING until |XTE| <= dRecov (2.0m)
                if (d <= cfg_.dRecov) {
                    navState_ = NavigationState::ON_TRAIL;
                }
            } else if (d > cfg_.dDev) {
                // In ON_TRAIL or DEVIATING: 3.0m < |XTE| <= 5.0m -> DEVIATING
                navState_ = NavigationState::DEVIATING;
            } else {
                // |XTE| <= 3.0m -> ON_TRAIL
                navState_ = NavigationState::ON_TRAIL;
            }
        }
    }

    out.state = navState_;
    out.offTrail = (navState_ == NavigationState::OFF_TRAIL);

    // 6. M6 Guidance Mode Selection & Guidance Vector (§18.1–§18.4, M6 Spec)
    P2 g{0.0, 0.0};

    if (navState_ == NavigationState::RETURN_COMPLETE) {
        out.guidanceMode = GuidanceMode::ARRIVED;
        out.recoveryActive = false;
        out.recoveryTarget = P2{p0.east, p0.north};
        out.guidanceVector = {0.0, 0.0};
        out.desiredHeading = psiU;
        out.steeringErr = 0.0;
        out.turnCommand = TurnCommand::STRAIGHT;
    } else if (navState_ == NavigationState::OFF_TRAIL || navState_ == NavigationState::RECOVERING) {
        // DIRECT_RECOVERY: guidance target = projected point Q on active recovery segment
        out.guidanceMode = GuidanceMode::DIRECT_RECOVERY;
        out.recoveryActive = true;
        out.recoveryTarget = q;
        P2 dvec = q - pu;
        if (norm(dvec) < 1e-6) {
            dvec = t;
        }
        g = dvec * (1.0 / std::max(norm(dvec), 1e-9));
        out.guidanceVector = g;
    } else if (navState_ == NavigationState::DEVIATING) {
        // LATERAL_BLEND: g = normalize(t_r + clamp(k_e * e_perp, -2.0, +2.0) * n_left)
        out.guidanceMode = GuidanceMode::LATERAL_BLEND;
        out.recoveryActive = false;
        out.recoveryTarget = q;
        const double lat = std::clamp(cfg_.ke * out.xte, -cfg_.keCap, cfg_.keCap);
        const P2 gb = t + nLeft * lat;
        g = gb * (1.0 / std::max(norm(gb), 1e-9));
        out.guidanceVector = g;
    } else { // NavigationState::ON_TRAIL
        // NORMAL_CARROT: guidance target = look-ahead carrot C
        out.guidanceMode = GuidanceMode::NORMAL_CARROT;
        out.recoveryActive = false;
        out.recoveryTarget = c;
        P2 dvec = c - pu;
        if (norm(dvec) < 1e-6) {
            dvec = t;
        }
        g = dvec * (1.0 / std::max(norm(dvec), 1e-9));
        out.guidanceVector = g;
    }

    // 7. Desired Heading, Steering Error (with M5 180° tie-break), and Turn Classification
    if (out.guidanceMode != GuidanceMode::ARRIVED) {
        out.desiredHeading = bearing(g);
        double err = wrapPi(out.desiredHeading - psiU);

        // Deterministic 180° tie-break policy preserved from M5 (§18.4)
        if (std::abs(std::abs(err) - M_PI) < cfg_.piEps) {
            err = (prevTurn_ >= 0 ? +M_PI : -M_PI);
        } else if (std::abs(err) > 0.2) {
            prevTurn_ = (err >= 0 ? +1 : -1);
        }
        out.steeringErr = err;
        out.turnCommand = classifyTurnCommand(err);
    }

    latestGuidance_ = out;
    return out;
}

} // namespace breadcrumb
