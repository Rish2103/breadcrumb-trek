#pragma once

#include "MathTypes.hpp"
#include <cmath>
#include <deque>
#include <algorithm>

/**
 * Windowed stationary detector and gyro-bias recalibration per gpt_technical_final.md §10.
 *
 * Evaluates over a window of T_stationary:
 *   | ||a|| - g | < epsA
 *   ||omega|| < epsG
 *   std(||a||) < sigmaA
 *   std(||omega||) < sigmaG
 */
class StationaryDetector {
public:
    struct Config {
        float gRef = 9.80665f;       // Earth standard gravity (m/s²)
        float epsA = 0.60f;          // Accel magnitude deviation threshold (m/s²)
        float epsG = 0.15f;          // Angular velocity norm threshold (rad/s)
        float sigmaA = 0.15f;        // Accel standard deviation threshold (m/s²)
        float sigmaG = 0.05f;        // Gyro standard deviation threshold (rad/s)
        float alpha = 0.02f;         // Bias exponential moving average rate
        int   minSamples = 30;       // Minimum stationary window samples (~0.6s at 50 Hz)
    };

    Config cfg;
    Vec3 gyroBias{0.0f, 0.0f, 0.0f};

    /**
     * Updates detector with latest accel and gyro measurements.
     */
    void update(float ax, float ay, float az,
                float gx, float gy, float gz) {

        const float aNorm = std::sqrt(ax * ax + ay * ay + az * az);
        const float gNorm = std::sqrt(gx * gx + gy * gy + gz * gz);

        aBuffer_.push_back(aNorm);
        gBuffer_.push_back(gNorm);

        if (aBuffer_.size() > static_cast<size_t>(cfg.minSamples)) {
            aBuffer_.pop_front();
            gBuffer_.pop_front();
        }

        if (aBuffer_.size() < static_cast<size_t>(cfg.minSamples)) {
            isStationary_ = false;
            return;
        }

        // Calculate statistics
        float aMean = 0.0f, gMean = 0.0f;
        for (size_t i = 0; i < aBuffer_.size(); ++i) {
            aMean += aBuffer_[i];
            gMean += gBuffer_[i];
        }
        aMean /= aBuffer_.size();
        gMean /= gBuffer_.size();

        float aVar = 0.0f, gVar = 0.0f;
        for (size_t i = 0; i < aBuffer_.size(); ++i) {
            aVar += (aBuffer_[i] - aMean) * (aBuffer_[i] - aMean);
            gVar += (gBuffer_[i] - gMean) * (gBuffer_[i] - gMean);
        }
        const float aStd = std::sqrt(aVar / aBuffer_.size());
        const float gStd = std::sqrt(gVar / gBuffer_.size());

        // Multi-sensor windowed stationary criteria
        const bool aMagOk = std::abs(aMean - cfg.gRef) < cfg.epsA;
        const bool gMagOk = gMean < cfg.epsG;
        const bool aStdOk = aStd < cfg.sigmaA;
        const bool gStdOk = gStd < cfg.sigmaG;

        if (aMagOk && gMagOk && aStdOk && gStdOk) {
            isStationary_ = true;
            // Bias recalibration: b̂_g ← (1 - α)·b̂_g + α·ω_m
            gyroBias.x = (1.0f - cfg.alpha) * gyroBias.x + cfg.alpha * gx;
            gyroBias.y = (1.0f - cfg.alpha) * gyroBias.y + cfg.alpha * gy;
            gyroBias.z = (1.0f - cfg.alpha) * gyroBias.z + cfg.alpha * gz;
        } else {
            isStationary_ = false;
        }
    }

    bool isStationary() const { return isStationary_; }

    float getBiasNorm() const {
        return std::sqrt(gyroBias.x * gyroBias.x + gyroBias.y * gyroBias.y + gyroBias.z * gyroBias.z);
    }

    void reset() {
        aBuffer_.clear();
        gBuffer_.clear();
        gyroBias = {0.0f, 0.0f, 0.0f};
        isStationary_ = false;
    }

private:
    std::deque<float> aBuffer_;
    std::deque<float> gBuffer_;
    bool isStationary_ = false;
};
