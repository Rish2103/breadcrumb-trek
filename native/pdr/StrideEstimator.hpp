#pragma once

#include <algorithm>
#include <cmath>

/**
 * Bounded Weinberg-style stride length estimation per gpt_technical_final.md §12.
 *
 *   L_k = clamp( K_mode · (A_max,k - A_min,k)^(1/4), L_min, L_max )
 *
 * K_mode is explicitly configurable (user/motion calibration parameter, not a universal constant).
 */
class StrideEstimator {
public:
    struct Config {
        float K = 0.42f;            // Calibration constant (typically 0.38 - 0.48 depending on user height)
        float minStride = 0.30f;    // Physical lower bound (m)
        float maxStride = 1.20f;    // Physical upper bound (m)
    };

    Config cfg;

    /**
     * Estimates stride length from peak-to-peak vertical bounce acceleration.
     *
     * @param aMax Maximum acceleration magnitude during the step cycle (m/s²)
     * @param aMin Minimum acceleration magnitude during the step cycle (m/s²)
     * @return Stride length in meters, clamped within [minStride, maxStride]
     */
    float estimate(float aMax, float aMin) const {
        const float bounce = std::max(0.0f, aMax - aMin);
        const float rawStride = cfg.K * std::pow(bounce, 0.25f);
        return std::clamp(rawStride, cfg.minStride, cfg.maxStride);
    }
};
