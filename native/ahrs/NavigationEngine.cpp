#include "NavigationEngine.hpp"
#include <algorithm>

void NavigationEngine::pushAccel(float x, float y, float z, int64_t tNs) {
    latestAccel_ = {x, y, z};
    lastAccelT_ = tNs;

    const bool isStat = statDet_.isStationary();
    const float currentHeading = static_cast<float>(phoneForwardBearing(ahrs_.q));

    // Earth-frame horizontal acceleration for ConstrainedCourseObserver
    const Vec3 aEarth = rotateBodyToEarth(ahrs_.q, latestAccel_);
    const double aEast = -static_cast<double>(aEarth.y);
    const double aNorth = static_cast<double>(aEarth.x);

    const bool stepDetected = pdr_.update(x, y, z, tNs, currentHeading, isStat, aEast, aNorth);

    if (stepDetected && trail_.getState() == TrailRecordingState::RECORDING) {
        trail_.addPoint(tNs, pdr_.getEast(), pdr_.getNorth(), currentHeading,
                        pdr_.getStepCount(), pdr_.getCumulativeDistance(), magRel_.getCmag(),
                        pdr_.getLastStepHeading());
    }

    if (reverseNav_.isActive()) {
        const double speed = static_cast<double>(pdr_.getStepFrequency() * pdr_.getLastStride());
        const double userSpeed = (speed > 0.1) ? speed : 1.2;
        breadcrumb::P2 pu{static_cast<double>(pdr_.getEast()), static_cast<double>(pdr_.getNorth())};
        reverseNav_.update(pu, static_cast<double>(currentHeading), userSpeed);
    }
}

void NavigationEngine::pushMag(float x, float y, float z, int64_t tNs) {
    latestMag_ = {x, y, z};
    lastMagT_ = tNs;
    const bool isStat = statDet_.isStationary();
    magRel_.update(x, y, z, ahrs_.q, isStat);
}

void NavigationEngine::pushGyro(float x, float y, float z, int64_t tNs) {
    latestGyro_ = {x, y, z};

    if (!init_) {
        init_ = true;
        lastGyroT_ = tNs;
        return;
    }

    const double dt = static_cast<double>(tNs - lastGyroT_) * 1e-9;
    lastGyroT_ = tNs;

    // Guard against non-monotonic timestamps and large jumps per §7.3
    if (dt <= 0.0 || dt > 0.25) return;

    // Staleness checks:
    // Accel older than 100 ms -> treat as missing (zero vector)
    // Mag older than 300 ms -> treat as missing (Cmag = 0)
    const bool accelFresh = (tNs - lastAccelT_) < 100000000LL;
    const bool magFresh   = (tNs - lastMagT_)   < 300000000LL;

    const Vec3 a = accelFresh ? latestAccel_ : Vec3{0.0f, 0.0f, 0.0f};
    const Vec3 m = magFresh   ? latestMag_   : Vec3{0.0f, 0.0f, 0.0f};
    const float Cmag = magFresh ? magRel_.getCmag() : 0.0f;

    // Stationary detector and gyro bias recalibration (§10)
    statDet_.update(a.x, a.y, a.z, x, y, z);
    const Vec3 bias = statDet_.gyroBias;

    // Bias-corrected angular velocity
    const float gcx = x - bias.x;
    const float gcy = y - bias.y;
    const float gcz = z - bias.z;

    // AHRS MARG update
    ahrs_.update(gcx, gcy, gcz, a.x, a.y, a.z, m.x, m.y, m.z, Cmag, static_cast<float>(dt));
}

bool NavigationEngine::startTrailRecording() {
    pdr_.reset();
    const float currentHeading = static_cast<float>(phoneForwardBearing(ahrs_.q));
    trail_.startRecording(lastAccelT_ > 0 ? lastAccelT_ : 1000000000LL, currentHeading, currentHeading);
    return true;
}

bool NavigationEngine::stopTrailRecording() {
    trail_.stopRecording(lastAccelT_ > 0 ? lastAccelT_ : 1000000000LL);
    return true;
}

void NavigationEngine::resetTrail() {
    pdr_.reset();
    trail_.reset();
    reverseNav_.stopReturn();
}

int NavigationEngine::getTrailPoints(float* outFloats, int maxPoints) {
    return trail_.copyPointsToBuffer(outFloats, maxPoints);
}

bool NavigationEngine::startReverseNavigation() {
    return reverseNav_.startReturn(trail_);
}

void NavigationEngine::stopReverseNavigation() {
    reverseNav_.stopReturn();
}

bool NavigationEngine::isReverseNavigationActive() const {
    return reverseNav_.isActive();
}

breadcrumb::GuidanceOut NavigationEngine::getGuidanceOutput() const {
    return reverseNav_.getLatestGuidance();
}

int NavigationEngine::getState(float* outBuffer, int size) {
    if (outBuffer == nullptr || size < 12) return -1;

    // Refresh guidance for current position and heading if active
    const float currentHeading = static_cast<float>(phoneForwardBearing(ahrs_.q));
    if (reverseNav_.isActive()) {
        const double speed = static_cast<double>(pdr_.getStepFrequency() * pdr_.getLastStride());
        const double userSpeed = (speed > 0.1) ? speed : 1.2;
        breadcrumb::P2 pu{static_cast<double>(pdr_.getEast()), static_cast<double>(pdr_.getNorth())};
        reverseNav_.update(pu, static_cast<double>(currentHeading), userSpeed);
    }

    // State layout:
    // 0: E (m East)
    // 1: N (m North)
    // 2: U (0.0f)
    // 3: heading (rad, clockwise from North)
    // 4: qw
    // 5: qx
    // 6: qy
    // 7: qz
    // 8: step_count
    // 9: stride_length (m)
    // 10: cumulative_distance (m)
    // 11: step_frequency (Hz)
    // 12: is_stationary (1.0f if stationary, 0.0f if moving/walking)
    // 13: Cmag (magnetic reliability)
    // 14: gyro_bias_norm (rad/s)
    // 15: calibrated_B0 (µT, or -1.0f if uncalibrated)
    // 16: trail_point_count
    // 17: trail_recording_state (0=IDLE, 1=RECORDING, 2=RETURN_READY)
    // 18: journey_duration_sec
    // 19: total_trail_distance
    // 20: is_reverse_nav_active (1.0f or 0.0f)
    // 21: reverse_progress_index
    // 22: matched_segment_index
    // 23: proj_point_east
    // 24: proj_point_north
    // 25: carrot_east
    // 26: carrot_north
    // 27: cross_track_error (signed meters, + = trail left, - = trail right)
    // 28: desired_heading_rad
    // 29: steering_error_rad
    // 30: nav_state_code (0=ON_TRAIL, 1=DEVIATING, 2=OFF_TRAIL, 3=RECOVERING, 4=RETURN_COMPLETE)
    // 31: return_completed (1.0f or 0.0f)
    // 32: guidance_mode_code (0=NORMAL_CARROT, 1=LATERAL_BLEND, 2=DIRECT_RECOVERY, 3=ARRIVED)
    // 33: turn_command_code (0=STRAIGHT, 1=TURN_LEFT, 2=TURN_RIGHT, 3=U_TURN)
    // 34: recovery_active (1.0f or 0.0f)
    // 35: recovery_target_east
    // 36: recovery_target_north
    // 37: guidance_vector_east
    // 38: guidance_vector_north
    // 39: reserved (0.0f)

    outBuffer[0] = pdr_.getEast();
    outBuffer[1] = pdr_.getNorth();
    outBuffer[2] = 0.0f;
    outBuffer[3] = currentHeading;
    outBuffer[4] = ahrs_.q.w;
    outBuffer[5] = ahrs_.q.x;
    outBuffer[6] = ahrs_.q.y;
    outBuffer[7] = ahrs_.q.z;

    if (size >= 39) {
        outBuffer[8] = static_cast<float>(pdr_.getStepCount());
        outBuffer[9] = pdr_.getLastStride();
        outBuffer[10] = pdr_.getCumulativeDistance();
        outBuffer[11] = pdr_.getStepFrequency();
        outBuffer[12] = statDet_.isStationary() ? 1.0f : 0.0f;
        outBuffer[13] = magRel_.getCmag();
        outBuffer[14] = statDet_.getBiasNorm();
        outBuffer[15] = magRel_.isCalibrated() ? magRel_.getB0() : -1.0f;
        outBuffer[16] = static_cast<float>(trail_.getPointCount());
        outBuffer[17] = static_cast<float>(static_cast<int>(trail_.getState()));
        outBuffer[18] = static_cast<float>(trail_.getJourneyDurationSec(lastAccelT_));
        outBuffer[19] = trail_.getTotalDistance();

        const auto& g = reverseNav_.getLatestGuidance();
        outBuffer[20] = reverseNav_.isActive() ? 1.0f : 0.0f;
        outBuffer[21] = static_cast<float>(g.reverseProgressIndex);
        outBuffer[22] = static_cast<float>(g.segIndex);
        outBuffer[23] = static_cast<float>(g.projPoint.e);
        outBuffer[24] = static_cast<float>(g.projPoint.n);
        outBuffer[25] = static_cast<float>(g.carrotPoint.e);
        outBuffer[26] = static_cast<float>(g.carrotPoint.n);
        outBuffer[27] = static_cast<float>(g.xte);
        outBuffer[28] = static_cast<float>(g.desiredHeading);
        outBuffer[29] = static_cast<float>(g.steeringErr);
        outBuffer[30] = static_cast<float>(static_cast<uint32_t>(g.state));
        outBuffer[31] = g.returnComplete ? 1.0f : 0.0f;
        outBuffer[32] = static_cast<float>(static_cast<uint32_t>(g.guidanceMode));
        outBuffer[33] = static_cast<float>(static_cast<uint32_t>(g.turnCommand));
        outBuffer[34] = g.recoveryActive ? 1.0f : 0.0f;
        outBuffer[35] = static_cast<float>(g.recoveryTarget.e);
        outBuffer[36] = static_cast<float>(g.recoveryTarget.n);
        outBuffer[37] = static_cast<float>(g.guidanceVector.e);
        outBuffer[38] = static_cast<float>(g.guidanceVector.n);
        if (size >= 40) {
            outBuffer[39] = pdr_.getLastStepHeading(); // travel heading (rad)
            return 40;
        }
        return 39;
    } else if (size >= 32) {
        outBuffer[8] = static_cast<float>(pdr_.getStepCount());
        outBuffer[9] = pdr_.getLastStride();
        outBuffer[10] = pdr_.getCumulativeDistance();
        outBuffer[11] = pdr_.getStepFrequency();
        outBuffer[12] = statDet_.isStationary() ? 1.0f : 0.0f;
        outBuffer[13] = magRel_.getCmag();
        outBuffer[14] = statDet_.getBiasNorm();
        outBuffer[15] = magRel_.isCalibrated() ? magRel_.getB0() : -1.0f;
        outBuffer[16] = static_cast<float>(trail_.getPointCount());
        outBuffer[17] = static_cast<float>(static_cast<int>(trail_.getState()));
        outBuffer[18] = static_cast<float>(trail_.getJourneyDurationSec(lastAccelT_));
        outBuffer[19] = trail_.getTotalDistance();

        const auto& g = reverseNav_.getLatestGuidance();
        outBuffer[20] = reverseNav_.isActive() ? 1.0f : 0.0f;
        outBuffer[21] = static_cast<float>(g.reverseProgressIndex);
        outBuffer[22] = static_cast<float>(g.segIndex);
        outBuffer[23] = static_cast<float>(g.projPoint.e);
        outBuffer[24] = static_cast<float>(g.projPoint.n);
        outBuffer[25] = static_cast<float>(g.carrotPoint.e);
        outBuffer[26] = static_cast<float>(g.carrotPoint.n);
        outBuffer[27] = static_cast<float>(g.xte);
        outBuffer[28] = static_cast<float>(g.desiredHeading);
        outBuffer[29] = static_cast<float>(g.steeringErr);
        outBuffer[30] = static_cast<float>(static_cast<uint32_t>(g.state));
        outBuffer[31] = g.returnComplete ? 1.0f : 0.0f;
        return 32;
    } else if (size >= 20) {
        outBuffer[8] = static_cast<float>(pdr_.getStepCount());
        outBuffer[9] = pdr_.getLastStride();
        outBuffer[10] = pdr_.getCumulativeDistance();
        outBuffer[11] = pdr_.getStepFrequency();
        outBuffer[12] = statDet_.isStationary() ? 1.0f : 0.0f;
        outBuffer[13] = magRel_.getCmag();
        outBuffer[14] = statDet_.getBiasNorm();
        outBuffer[15] = magRel_.isCalibrated() ? magRel_.getB0() : -1.0f;
        outBuffer[16] = static_cast<float>(trail_.getPointCount());
        outBuffer[17] = static_cast<float>(static_cast<int>(trail_.getState()));
        outBuffer[18] = static_cast<float>(trail_.getJourneyDurationSec(lastAccelT_));
        outBuffer[19] = trail_.getTotalDistance();
        return 20;
    } else if (size >= 16) {
        outBuffer[8] = static_cast<float>(pdr_.getStepCount());
        outBuffer[9] = pdr_.getLastStride();
        outBuffer[10] = pdr_.getCumulativeDistance();
        outBuffer[11] = pdr_.getStepFrequency();
        outBuffer[12] = statDet_.isStationary() ? 1.0f : 0.0f;
        outBuffer[13] = magRel_.getCmag();
        outBuffer[14] = statDet_.getBiasNorm();
        outBuffer[15] = magRel_.isCalibrated() ? magRel_.getB0() : -1.0f;
        return 16;
    } else {
        // Backwards compatibility for 12-element buffer
        outBuffer[8] = magRel_.getCmag();
        outBuffer[9] = statDet_.isStationary() ? 1.0f : 0.0f;
        outBuffer[10] = statDet_.getBiasNorm();
        outBuffer[11] = magRel_.isCalibrated() ? magRel_.getB0() : -1.0f;
        return 12;
    }
}

void NavigationEngine::reset() {
    ahrs_.q = {1.0f, 0.0f, 0.0f, 0.0f};
    magRel_.reset();
    statDet_.reset();
    pdr_.reset();
    trail_.reset();
    reverseNav_.stopReturn();
    latestAccel_ = {0.0f, 0.0f, 0.0f};
    latestGyro_ = {0.0f, 0.0f, 0.0f};
    latestMag_ = {0.0f, 0.0f, 0.0f};
    lastAccelT_ = 0;
    lastGyroT_ = 0;
    lastMagT_ = 0;
    init_ = false;
}
