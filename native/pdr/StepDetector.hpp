#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

/**
 * Deterministic walking step detector per gpt_technical_final.md §11.
 *
 * Signal processing pipeline:
 *   ‖a‖ → slow-baseline removal (HPF) + low-pass filter (LPF) → band-limited gait signal s0
 *       → adaptive threshold: T = clamp(μ + k·σ, thrMin, thrMax)
 *       → 3-sample peak candidate check (s1 > s2 && s1 >= s0 && s1 > T)
 *       → prominence check (s1 - valleyMin > minProminence)
 *       → interval constraints (minIntervalS <= interval <= maxIntervalS)
 *       → confirmed step event
 */
class StepDetector {
public:
    struct Config {
        float hpHz = 0.5f;            // Baseline removal cutoff (~0.5 Hz)
        float lpHz = 4.0f;            // Gait bandpass upper cutoff (~4.0 Hz)
        float k = 0.5f;               // Adaptive threshold sensitivity factor
        float thrMin = 0.40f;         // Minimum adaptive threshold (m/s²)
        float thrMax = 3.50f;         // Maximum adaptive threshold (m/s²)
        float minProminence = 0.50f;  // Peak prominence threshold (m/s²)
        float minIntervalS = 0.25f;   // Minimum step interval (0.25s -> max 4 Hz cadence)
        float maxIntervalS = 1.50f;   // Maximum interval for cadence tracking (1.50s)
    };

    struct Step {
        bool detected = false;
        int64_t tNs = 0;
        float aMax = 0.0f;
        float aMin = 0.0f;
        float confidence = 0.0f;
        float intervalS = 0.0f;
    };

    Config cfg;

    /**
     * Ingests a new acceleration magnitude sample.
     *
     * @param aMag Raw acceleration norm ‖a_b‖ in m/s²
     * @param tNs Monotonic timestamp in nanoseconds
     * @param isStationary Gating flag: if true, step counting is disabled
     * @return Step event details
     */
    Step push(float aMag, int64_t tNs, bool isStationary = false) {
        Step out;

        // While confidently stationary, reject steps and reset peak state (§11)
        if (isStationary) {
            valleyMin_ = 0.0f;
            s1_ = 0.0f;
            s2_ = 0.0f;
            rawMax_ = aMag;
            rawMin_ = aMag;
            return out;
        }

        if (!init_) {
            init_ = true;
            slow_ = aMag;
            lp_ = 0.0f;
            lastT_ = tNs;
            tPrev_ = tNs;
            rawMax_ = aMag;
            rawMin_ = aMag;
            valleyMin_ = 0.0f;
            return out;
        }

        const float dt = static_cast<float>(tNs - lastT_) * 1e-9f;
        lastT_ = tNs;

        // Guard against non-monotonic timestamps and abnormal gaps
        if (dt <= 0.0f || dt > 0.25f) {
            return out;
        }

        constexpr float TWO_PI = 6.283185307179586f;
        const float aHp = dt / (1.0f / (TWO_PI * cfg.hpHz) + dt);
        const float aLp = dt / (1.0f / (TWO_PI * cfg.lpHz) + dt);

        slow_ += aHp * (aMag - slow_);           // Slow baseline tracking gravity (~9.81 m/s²)
        lp_   += aLp * ((aMag - slow_) - lp_);   // Band-limited gait dynamic acceleration
        const float s0 = lp_;

        rawMax_ = std::max(rawMax_, aMag);
        rawMin_ = std::min(rawMin_, aMag);
        valleyMin_ = std::min(valleyMin_, s0);

        // Adaptive statistics via per-sample EMA
        constexpr float aS = 0.02f;
        mu_  += aS * (s0 - mu_);
        var_ += aS * ((s0 - mu_) * (s0 - mu_) - var_);
        const float sigma = std::sqrt(std::max(0.0f, var_));
        const float thr = std::clamp(mu_ + cfg.k * sigma, cfg.thrMin, cfg.thrMax);

        // Peak candidate at middle sample (s1_) of 3-point window
        if (s1_ > s2_ && s1_ >= s0 && s1_ > thr && (s1_ - valleyMin_) > cfg.minProminence) {
            const float interval = (lastStepT_ > 0) ? static_cast<float>(tPrev_ - lastStepT_) * 1e-9f : 1e9f;

            if (interval >= cfg.minIntervalS) {
                float conf = 0.40f;
                if (emaInt_ > 0.0f && interval <= cfg.maxIntervalS) {
                    conf = std::exp(-2.0f * std::abs(interval - emaInt_) / emaInt_);
                }

                if (interval <= cfg.maxIntervalS) {
                    emaInt_ = (emaInt_ > 0.0f) ? (0.80f * emaInt_ + 0.20f * interval) : interval;
                }

                out.detected = true;
                out.tNs = tPrev_;
                out.aMax = rawMax_;
                out.aMin = rawMin_;
                out.confidence = std::clamp(conf, 0.0f, 1.0f);
                out.intervalS = (interval <= cfg.maxIntervalS) ? interval : 0.0f;

                lastStepT_ = tPrev_;
                rawMax_ = aMag;
                rawMin_ = aMag;
                valleyMin_ = s0;
            }
        }

        s2_ = s1_;
        s1_ = s0;
        tPrev_ = tNs;

        return out;
    }

    void reset() {
        init_ = false;
        slow_ = 0.0f;
        lp_ = 0.0f;
        mu_ = 0.0f;
        var_ = 0.0f;
        s1_ = 0.0f;
        s2_ = 0.0f;
        rawMax_ = 0.0f;
        rawMin_ = 0.0f;
        valleyMin_ = 0.0f;
        emaInt_ = 0.0f;
        lastT_ = 0;
        tPrev_ = 0;
        lastStepT_ = 0;
    }

private:
    bool init_ = false;
    float slow_ = 0.0f;
    float lp_ = 0.0f;
    float mu_ = 0.0f;
    float var_ = 0.0f;
    float s1_ = 0.0f;
    float s2_ = 0.0f;
    float rawMax_ = 0.0f;
    float rawMin_ = 0.0f;
    float valleyMin_ = 0.0f;
    float emaInt_ = 0.0f;

    int64_t lastT_ = 0;
    int64_t tPrev_ = 0;
    int64_t lastStepT_ = 0;
};
