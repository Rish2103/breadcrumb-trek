#pragma once

#include "MathTypes.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

/**
 * Continuous magnetic field reliability estimator per gpt_technical_final.md §9.
 * Computes Cmag ∈ [0, 1] using magnitude deviation from local B0,
 * rolling variance, and dip-angle consistency.
 *
 * Does NOT use a fixed global baseline (no hardcoded B0).
 * Calibrates local B0 and dip0 during an initial stationary calibration period.
 * If calibration is unavailable or insufficient, uses conservative fallback: Cmag = 0.0.
 */
class MagneticReliability {
public:
    enum class CalibrationStatus {
        UNAVAILABLE,
        CALIBRATING,
        CALIBRATED
    };

    struct Config {
        float lambdaB = 4.0f;       // Magnitude deviation sensitivity (§9.2)
        float lambdaSigma = 2.0f;   // Windowed variance sensitivity (§9.2)
        float lambdaDip = 3.0f;     // Dip angle deviation sensitivity (§9.2)
        int   windowSize = 20;      // Number of mag samples for variance tracking (~1.0s at 20 Hz)
        int   minCalibSamples = 20; // Minimum stationary samples required to establish baseline (~1.0s at 20 Hz)
        float maxCalibStd = 3.0f;   // Maximum allowed std during calibration (µT) to reject disturbed periods
    };

    Config cfg;

    /**
     * Feeds a magnetometer sample during stationary calibration.
     * Returns true if calibration completed successfully with this sample.
     */
    bool feedCalibrationSample(float mx, float my, float mz, const Quaternion& q) {
        if (isCalibrated_) return true;

        const float B = std::sqrt(mx * mx + my * my + mz * mz);
        if (std::isnan(B) || std::isinf(B) || B < 1.0e-3f) {
            return false;
        }

        // Earth-frame magnetic dip (angle between Earth-frame magnetic vector and Earth Up z_e)
        const Vec3 mEarth = rotateBodyToEarth(q, {mx, my, mz});
        const float dip = std::acos(std::clamp(mEarth.z / B, -1.0f, 1.0f));

        calibSamplesB_.push_back(B);
        calibSamplesDip_.push_back(dip);

        if (static_cast<int>(calibSamplesB_.size()) >= cfg.minCalibSamples) {
            return finalizeCalibration();
        }
        return false;
    }

    /**
     * Finalizes calibration if sufficient and stable samples exist.
     */
    bool finalizeCalibration() {
        if (static_cast<int>(calibSamplesB_.size()) < cfg.minCalibSamples) {
            // Insufficient samples -> remains UNAVAILABLE
            return false;
        }

        float sumB = 0.0f;
        for (float b : calibSamplesB_) sumB += b;
        const float meanB = sumB / calibSamplesB_.size();

        float varB = 0.0f;
        for (float b : calibSamplesB_) varB += (b - meanB) * (b - meanB);
        const float stdB = std::sqrt(varB / calibSamplesB_.size());

        // Reject calibration if local magnetic field was unstable/fluctuating
        if (stdB > cfg.maxCalibStd) {
            calibSamplesB_.clear();
            calibSamplesDip_.clear();
            return false;
        }

        float sumDip = 0.0f;
        for (float d : calibSamplesDip_) sumDip += d;
        const float meanDip = sumDip / calibSamplesDip_.size();

        B0_ = meanB;
        dip0_ = meanDip;
        isCalibrated_ = true;
        calibSamplesB_.clear();
        calibSamplesDip_.clear();
        return true;
    }

    /**
     * Updates magnetic reliability with latest magnetometer sample and Earth-frame dip angle.
     *
     * If uncalibrated:
     *   - If isStationary == true, attempts to accumulate calibration samples.
     *   - If calibration is not yet established, returns conservative fallback Cmag = 0.0.
     *
     * If calibrated:
     *   - Computes continuous Cmag ∈ [0, 1] based on deviation from calibrated B0 and dip0.
     */
    float update(float mx, float my, float mz, const Quaternion& q, bool isStationary = false) {
        const float B = std::sqrt(mx * mx + my * my + mz * mz);
        if (std::isnan(B) || std::isinf(B) || B < 1.0e-3f) {
            lastCmag_ = 0.0f;
            return 0.0f;
        }

        // If not calibrated, attempt calibration only during stationary hold
        if (!isCalibrated_) {
            if (isStationary) {
                feedCalibrationSample(mx, my, mz, q);
            } else {
                // If motion occurs before calibration completes, reset incomplete buffer
                calibSamplesB_.clear();
                calibSamplesDip_.clear();
            }

            if (!isCalibrated_) {
                // Conservative fallback when calibration is unavailable or insufficient:
                // Cmag = 0.0 disables magnetic gradient correction, preventing false heading pull.
                lastCmag_ = 0.0f;
                return 0.0f;
            }
        }

        // Maintain sliding window for variance calculation
        window_.push_back(B);
        if (window_.size() > static_cast<size_t>(cfg.windowSize)) {
            window_.pop_front();
        }

        float mean = 0.0f;
        for (float v : window_) mean += v;
        mean /= window_.size();

        float var = 0.0f;
        for (float v : window_) var += (v - mean) * (v - mean);
        const float windowStd = std::sqrt(var / window_.size());

        // Compute dip angle: angle between Earth-frame magnetic vector and Earth Up (z_e)
        const Vec3 mEarth = rotateBodyToEarth(q, {mx, my, mz});
        const float dip = std::acos(std::clamp(mEarth.z / B, -1.0f, 1.0f));

        // Strength deviation from local calibrated B0 (§9.2)
        const float dB = (B0_ > 1.0e-6f) ? std::abs(B - B0_) / B0_ : 1.0f;

        // Dip-angle deviation from local calibrated dip0 (§9.2)
        const float dDip = std::abs(dip - dip0_);

        // Continuous reliability exponential weighting (§9.2):
        // Cmag = exp(-λ_B · d_B) · exp(-λ_σ · σ_B / B0) · exp(-λ_dip · d_dip)
        const float c = std::exp(-cfg.lambdaB * dB) *
                        std::exp(-cfg.lambdaSigma * (windowStd / std::max(B0_, 1.0f))) *
                        std::exp(-cfg.lambdaDip * dDip);

        lastCmag_ = std::clamp(c, 0.0f, 1.0f);
        return lastCmag_;
    }

    float getCmag() const { return lastCmag_; }
    bool  isCalibrated() const { return isCalibrated_; }
    float getB0() const { return B0_; }
    float getDip0() const { return dip0_; }
    int   getCalibrationSampleCount() const { return static_cast<int>(calibSamplesB_.size()); }

    CalibrationStatus getCalibrationStatus() const {
        if (isCalibrated_) return CalibrationStatus::CALIBRATED;
        if (!calibSamplesB_.empty()) return CalibrationStatus::CALIBRATING;
        return CalibrationStatus::UNAVAILABLE;
    }

    void setReference(float referenceB0, float referenceDip0) {
        B0_ = referenceB0;
        dip0_ = referenceDip0;
        isCalibrated_ = true;
        calibSamplesB_.clear();
        calibSamplesDip_.clear();
    }

    void reset() {
        window_.clear();
        calibSamplesB_.clear();
        calibSamplesDip_.clear();
        B0_ = 0.0f;
        dip0_ = 0.0f;
        isCalibrated_ = false;
        lastCmag_ = 0.0f; // Conservative fallback
    }

private:
    float B0_ = 0.0f;       // Calibrated local magnitude baseline (µT)
    float dip0_ = 0.0f;     // Calibrated local dip angle baseline (rad)
    bool  isCalibrated_ = false;
    float lastCmag_ = 0.0f; // Conservative fallback until calibrated

    std::deque<float> window_;
    std::vector<float> calibSamplesB_;
    std::vector<float> calibSamplesDip_;
};
