#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <deque>

#if __has_include("ahrs/MathTypes.hpp")
#include "ahrs/MathTypes.hpp"
#elif __has_include("MathTypes.hpp")
#include "MathTypes.hpp"
#elif __has_include("../ahrs/MathTypes.hpp")
#include "../ahrs/MathTypes.hpp"
#endif

/**
 * Production Constrained Pedestrian Travel-Course Observer per Milestone M3.8.
 *
 * Combines Earth-frame horizontal acceleration PCA with previous travel course
 * continuity, eigenvalue confidence weighting, bounded step changes, and a soft
 * phone-bearing prior to reject handheld yaw oscillations while tracking genuine translation.
 * Includes M3.8A dynamic walking-reversal detector and isolated transition limiter.
 */
class ConstrainedCourseObserver {
public:
    struct Config {
        float alpha = 0.35f;             // PCA innovation gain
        float betaPrior = 0.05f;         // Soft phone-forward prior gain
        float minPcaConfidence = 0.20f;  // Minimum eigenvalue separation to accept PCA
        float minLambda1 = 0.08f;        // Minimum dominant acceleration variance (m²/s⁴)
        float maxCourseChangeRad = 0.436f; // Max course change per step (~25°)
        float turnThresholdRad = 2.094f; // Persistent angle (>120°) indicating genuine turnaround

        // M3.8A Dynamic Walking-Reversal Parameters
        float reversalDivergenceThresholdRad = 2.44346f; // ~140° divergence threshold
        uint32_t reversalPersistenceSteps = 3;           // >= 3 consecutive steps
        float reversalMaxCourseChangeRad = 0.7854f;      // ~45°/step isolated transition limit
        float reversalExitDivergenceRad = 0.7854f;       // ~45° divergence to exit reversal mode
    };

    struct PcaOutput {
        bool valid = false;
        bool accepted = false;
        float confidence = 0.0f;
        float lambda1 = 0.0f;
        float lambda2 = 0.0f;
        float axisAngleRad = 0.0f;
        float orientedAngleRad = 0.0f;
    };

    Config cfg;

    ConstrainedCourseObserver() = default;

    void reset(float initialCourseRad = 0.0f, bool startInitialized = false) {
        currentCourseRad_ = initialCourseRad;
        initialized_ = startInitialized;
        totalUpdates_ = 0;
        acceptedPcaUpdates_ = 0;
        rejectedPcaUpdates_ = 0;
        signFlips180_ = 0;
        lastStepTimeNs_ = 0;
        consecutiveDivergentSteps_ = 0;
        reversalActive_ = false;
        reversalAnchorTarget_ = 0.0f;
        reversalTriggerStep_ = 0;
        lastPhoneDivergenceDeg_ = 0.0f;
    }

    float getTravelCourseRad() const { return currentCourseRad_; }
    float getTravelCourseDeg() const { return currentCourseRad_ * 180.0f / static_cast<float>(M_PI); }
    bool isInitialized() const { return initialized_; }
    uint32_t getTotalUpdates() const { return totalUpdates_; }
    uint32_t getAcceptedPcaUpdates() const { return acceptedPcaUpdates_; }
    uint32_t getRejectedPcaUpdates() const { return rejectedPcaUpdates_; }
    uint32_t getSignFlips180() const { return signFlips180_; }
    const PcaOutput& getLastPca() const { return lastPca_; }

    // M3.8A Reversal Telemetry
    bool isReversalActive() const { return reversalActive_; }
    uint32_t getConsecutiveDivergentSteps() const { return consecutiveDivergentSteps_; }
    uint32_t getReversalTriggerStep() const { return reversalTriggerStep_; }
    float getLastPhoneDivergenceDeg() const { return lastPhoneDivergenceDeg_; }

    /**
     * Ingests horizontal acceleration samples (aEast, aNorth) over a step interval
     * alongside current phone forward bearing and stationary flag.
     *
     * @param accelEastNorth Array of (aEast, aNorth) pairs in Earth navigation frame
     * @param phoneBearingRad Step-averaged phone forward bearing
     * @param isStationary Flag indicating if device is stationary
     * @param stepTurnaroundHint Flag if stationary turn preceded this step
     * @return Updated travel course in radians [-pi, pi]
     */
    float update(const std::vector<std::pair<double, double>>& accelEastNorth,
                 float phoneBearingRad,
                 bool isStationary,
                 bool stepTurnaroundHint = false) {

        totalUpdates_++;

        // Stationary Stabilization / Reset Handling
        if (isStationary) {
            consecutiveDivergentSteps_ = 0;
            reversalActive_ = false;
            return currentCourseRad_;
        }

        // First step initialization
        if (!initialized_) {
            currentCourseRad_ = phoneBearingRad;
            initialized_ = true;
            return currentCourseRad_;
        }

        // If a deliberate 180° turnaround occurred while stopped
        if (stepTurnaroundHint) {
            currentCourseRad_ = phoneBearingRad;
            consecutiveDivergentSteps_ = 0;
            reversalActive_ = false;
            return currentCourseRad_;
        }

        // 1. Compute 2x2 PCA on horizontal Earth acceleration
        lastPca_ = computePca(accelEastNorth);
        bool pcaUsable = (lastPca_.valid && lastPca_.confidence >= cfg.minPcaConfidence && lastPca_.lambda1 >= cfg.minLambda1);

        // 2. M3.8A Dynamic Walking-Reversal Detector
        float phoneDivergenceRad = std::abs(static_cast<float>(wrapPi(phoneBearingRad - currentCourseRad_)));
        lastPhoneDivergenceDeg_ = phoneDivergenceRad * 180.0f / static_cast<float>(M_PI);

        if (phoneDivergenceRad > cfg.reversalDivergenceThresholdRad) {
            consecutiveDivergentSteps_++;
        } else {
            consecutiveDivergentSteps_ = 0;
        }

        // Safety Gating: active steps (!isStationary), sustained persistence (>=3 steps), usable PCA
        if (consecutiveDivergentSteps_ >= cfg.reversalPersistenceSteps && pcaUsable && !reversalActive_) {
            reversalActive_ = true;
            reversalAnchorTarget_ = static_cast<float>(wrapPi(currentCourseRad_ + M_PI));
            reversalTriggerStep_ = totalUpdates_;
        }

        float deltaPca = 0.0f;
        float effectiveConfidence = 0.0f;

        if (pcaUsable) {
            lastPca_.accepted = true;
            acceptedPcaUpdates_++;
            effectiveConfidence = lastPca_.confidence;

            // 3. Resolve ±180° ambiguity against reference course:
            // Normally against previous travel course (currentCourseRad_).
            // When genuine walking reversal is active, temporarily resolve against reversalAnchorTarget_ (currentCourseRad + pi)
            float psiRef = reversalActive_ ? reversalAnchorTarget_ : currentCourseRad_;

            float deltaPlus = static_cast<float>(wrapPi(lastPca_.axisAngleRad - psiRef));
            float deltaMinus = static_cast<float>(wrapPi(lastPca_.axisAngleRad + M_PI - psiRef));

            float diffComp = std::abs(deltaPlus) - std::abs(deltaMinus);
            bool choosePlus = false;
            if (std::abs(diffComp) > 0.15f) {
                choosePlus = (std::abs(deltaPlus) < std::abs(deltaMinus));
            } else {
                // Near tie: resolve ambiguity using phone forward bearing
                float dPhonePlus = std::abs(static_cast<float>(wrapPi(lastPca_.axisAngleRad - phoneBearingRad)));
                float dPhoneMinus = std::abs(static_cast<float>(wrapPi(lastPca_.axisAngleRad + M_PI - phoneBearingRad)));
                choosePlus = (dPhonePlus <= dPhoneMinus);
            }

            if (choosePlus) {
                lastPca_.orientedAngleRad = static_cast<float>(wrapPi(lastPca_.axisAngleRad));
                deltaPca = static_cast<float>(wrapPi(lastPca_.orientedAngleRad - currentCourseRad_));
            } else {
                lastPca_.orientedAngleRad = static_cast<float>(wrapPi(lastPca_.axisAngleRad + M_PI));
                deltaPca = static_cast<float>(wrapPi(lastPca_.orientedAngleRad - currentCourseRad_));
            }
        } else {
            lastPca_.accepted = false;
            rejectedPcaUpdates_++;
            effectiveConfidence = 0.0f;
            deltaPca = 0.0f;
        }

        // 4. Soft Phone-Forward Prior & Reversal Transition
        float deltaPhone = static_cast<float>(wrapPi(phoneBearingRad - currentCourseRad_));

        if (reversalActive_) {
            // When reversing, align deltaPca sign with deltaPhone (which indicates rotation direction)
            if (std::abs(std::abs(deltaPca) - static_cast<float>(M_PI)) < 0.25f) {
                deltaPca = (deltaPhone >= 0.0f ? 1.0f : -1.0f) * std::abs(deltaPca);
            }
        } else if (std::abs(deltaPhone) > cfg.turnThresholdRad) {
            deltaPhone = (deltaPhone > 0.0f ? 1.0f : -1.0f) * cfg.maxCourseChangeRad;
        }

        // 5. Combined Bounded Course Update
        float stepLimit = reversalActive_ ? cfg.reversalMaxCourseChangeRad : cfg.maxCourseChangeRad;
        float courseInnovation = cfg.alpha * effectiveConfidence * deltaPca + cfg.betaPrior * deltaPhone;
        float clampedChange = std::clamp(courseInnovation, -stepLimit, stepLimit);

        float newCourse = static_cast<float>(wrapPi(currentCourseRad_ + clampedChange));

        // Check for 180° flips for metric tracking
        float netChange = std::abs(static_cast<float>(wrapPi(newCourse - currentCourseRad_)));
        if (std::abs(netChange - static_cast<float>(M_PI)) < 0.35f) {
            signFlips180_++;
        }

        currentCourseRad_ = newCourse;

        // Check exit condition for reversal mode: once course is aligned with new direction, return to normal bounded observer
        if (reversalActive_) {
            float exitDivergence = std::abs(static_cast<float>(wrapPi(phoneBearingRad - currentCourseRad_)));
            if (exitDivergence < cfg.reversalExitDivergenceRad) {
                reversalActive_ = false;
                consecutiveDivergentSteps_ = 0;
            }
        }

        return currentCourseRad_;
    }

private:
    float currentCourseRad_ = 0.0f;
    bool initialized_ = false;
    uint32_t totalUpdates_ = 0;
    uint32_t acceptedPcaUpdates_ = 0;
    uint32_t rejectedPcaUpdates_ = 0;
    uint32_t signFlips180_ = 0;
    int64_t lastStepTimeNs_ = 0;
    PcaOutput lastPca_;

    // M3.8A Dynamic Reversal State
    uint32_t consecutiveDivergentSteps_ = 0;
    bool reversalActive_ = false;
    float reversalAnchorTarget_ = 0.0f;
    uint32_t reversalTriggerStep_ = 0;
    float lastPhoneDivergenceDeg_ = 0.0f;

    PcaOutput computePca(const std::vector<std::pair<double, double>>& samples) {
        PcaOutput out;
        if (samples.size() < 5) return out;

        double meanE = 0.0, meanN = 0.0;
        for (const auto& s : samples) {
            meanE += s.first;
            meanN += s.second;
        }
        meanE /= samples.size();
        meanN /= samples.size();

        double c11 = 0.0, c12 = 0.0, c22 = 0.0;
        for (const auto& s : samples) {
            double dE = s.first - meanE;
            double dN = s.second - meanN;
            c11 += dE * dE;
            c12 += dE * dN;
            c22 += dN * dN;
        }
        c11 /= samples.size();
        c12 /= samples.size();
        c22 /= samples.size();

        double T = c11 + c22;
        double D = c11 * c22 - c12 * c12;
        double disc = std::max(0.0, (T * 0.5) * (T * 0.5) - D);
        double lambda1 = T * 0.5 + std::sqrt(disc);
        double lambda2 = T * 0.5 - std::sqrt(disc);

        double vE = 0.0, vN = 0.0;
        if (std::abs(c12) > 1e-6) {
            vE = lambda1 - c22;
            vN = c12;
        } else {
            if (c11 >= c22) { vE = 1.0; vN = 0.0; }
            else            { vE = 0.0; vN = 1.0; }
        }
        double norm = std::hypot(vE, vN);
        if (norm < 1e-12) return out;
        vE /= norm;
        vN /= norm;

        out.valid = true;
        out.lambda1 = static_cast<float>(lambda1);
        out.lambda2 = static_cast<float>(lambda2);
        constexpr float eps = 1e-4f;
        out.confidence = static_cast<float>((lambda1 - lambda2) / (lambda1 + lambda2 + eps));
        out.axisAngleRad = static_cast<float>(std::atan2(vE, vN));
        return out;
    }
};
