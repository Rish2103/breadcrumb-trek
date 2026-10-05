#pragma once

#include "StepDetector.hpp"
#include "StrideEstimator.hpp"
#include "MathTypes.hpp"
#include "ConstrainedCourseObserver.hpp"
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>
#include <utility>
#include <algorithm>

#if defined(__ANDROID__)
#include <android/log.h>
#define PDR_LOGI(...) __android_log_print(ANDROID_LOG_INFO, "BreadcrumbPDR", __VA_ARGS__)
#else
#include <cstdio>
#define PDR_LOGI(...) std::printf(__VA_ARGS__); std::printf("\n")
#endif

/**
 * Course Estimator Mode per Milestone M3.8.
 * Allows runtime switching between M3.1 circular-mean phone-heading baseline
 * and M3.8 ConstrainedCourseObserver for A/B diagnostics and regression testing.
 */
enum class CourseEstimatorMode {
    PHONE_HEADING_BASELINE = 0,
    CONSTRAINED_TRAVEL_COURSE = 1
};

/**
 * Pedestrian Dead Reckoning (PDR) 2D trajectory engine per gpt_technical_final.md §14.
 *
 * For each accepted step k with stride L_k and selected travel heading ψ_k (clockwise from North):
 *   ΔE_k = L_k · sin(ψ_k)
 *   ΔN_k = L_k · cos(ψ_k)
 *   E_k  = E_{k-1} + ΔE_k
 *   N_k  = N_{k-1} + ΔN_k
 */
class PdrEngine {
public:
    struct HeadingSample {
        int64_t tNs = 0;
        float headingRad = 0.0f;
    };

    struct EarthAccelSample {
        int64_t tNs = 0;
        double aEast = 0.0;
        double aNorth = 0.0;
    };

    StepDetector stepDetector;
    StrideEstimator strideEstimator;

    /**
     * Computes circular mean of a set of angles in radians.
     * sin_mean = (1/N) * sum(sin(theta_i))
     * cos_mean = (1/N) * sum(cos(theta_i))
     * mean_angle = atan2(sin_mean, cos_mean)
     */
    static inline float circularMean(const float* headingsRad, int count) {
        if (count <= 0 || headingsRad == nullptr) return 0.0f;
        double sinSum = 0.0;
        double cosSum = 0.0;
        for (int i = 0; i < count; ++i) {
            sinSum += std::sin(headingsRad[i]);
            cosSum += std::cos(headingsRad[i]);
        }
        const double sinMean = sinSum / count;
        const double cosMean = cosSum / count;
        if (std::abs(sinMean) < 1e-12 && std::abs(cosMean) < 1e-12) {
            return headingsRad[0];
        }
        return static_cast<float>(std::atan2(sinMean, cosMean));
    }

    // Mode Configuration
    void setCourseMode(CourseEstimatorMode mode) { courseMode_ = mode; }
    CourseEstimatorMode getCourseMode() const { return courseMode_; }
    ConstrainedCourseObserver& getCourseObserver() { return courseObserver_; }
    const ConstrainedCourseObserver& getCourseObserver() const { return courseObserver_; }

    /**
     * Overloaded ingest for backward compatibility (defaults to baseline phone heading).
     */
    bool update(float ax, float ay, float az, int64_t tNs, float currentHeadingRad, bool isStationary) {
        return update(ax, ay, az, tNs, currentHeadingRad, isStationary, 0.0, 0.0, false);
    }

    /**
     * Ingests a new acceleration sample with Earth-frame horizontal acceleration.
     */
    bool update(float ax, float ay, float az, int64_t tNs, float currentHeadingRad, bool isStationary,
                double aEast, double aNorth) {
        return update(ax, ay, az, tNs, currentHeadingRad, isStationary, aEast, aNorth, true);
    }

    /**
     * Primary update pipeline.
     */
    bool update(float ax, float ay, float az, int64_t tNs, float currentHeadingRad, bool isStationary,
                double aEast, double aNorth, bool hasEarthAccel) {

        latestTimeNs_ = tNs;
        const float aMag = std::sqrt(ax * ax + ay * ay + az * az);

        // Store heading sample in history (retain last 2.5s)
        while (!headingHistory_.empty() && (tNs - headingHistory_.front().tNs) > 2500000000LL) {
            headingHistory_.pop_front();
        }
        headingHistory_.push_back({tNs, currentHeadingRad});

        // Store Earth-frame acceleration in history (retain last 2.5s)
        if (hasEarthAccel) {
            while (!earthAccelHistory_.empty() && (tNs - earthAccelHistory_.front().tNs) > 2500000000LL) {
                earthAccelHistory_.pop_front();
            }
            earthAccelHistory_.push_back({tNs, aEast, aNorth});
        }

        StepDetector::Step step = stepDetector.push(aMag, tNs, isStationary);
        if (step.detected) {
            const float stride = strideEstimator.estimate(step.aMax, step.aMin);
            const float peakHeading = currentHeadingRad;

            // Compute circular-mean phone heading over step interval [lastStepTimeNs_, step.tNs]
            const int64_t intervalStart = (lastStepTimeNs_ > 0 && (step.tNs - lastStepTimeNs_) <= 2000000000LL)
                                          ? lastStepTimeNs_
                                          : (step.tNs - 500000000LL); // Default to last 0.5s if first step or pause > 2s
            const int64_t intervalEnd = step.tNs;

            const float stepPhoneHeading = computeIntervalHeading(intervalStart, intervalEnd, peakHeading);
            lastPhoneHeading_ = stepPhoneHeading;

            float selectedCourseHeading = stepPhoneHeading;

            if (courseMode_ == CourseEstimatorMode::CONSTRAINED_TRAVEL_COURSE && hasEarthAccel) {
                std::vector<std::pair<double, double>> winAcc;
                for (const auto& a : earthAccelHistory_) {
                    if (a.tNs >= intervalStart && a.tNs <= intervalEnd) {
                        winAcc.push_back({a.aEast, a.aNorth});
                    }
                }
                selectedCourseHeading = courseObserver_.update(winAcc, stepPhoneHeading, isStationary);
            } else {
                selectedCourseHeading = stepPhoneHeading;
            }
            lastTravelHeading_ = selectedCourseHeading;

            const float deltaHeading = static_cast<float>(wrapPi(selectedCourseHeading - peakHeading));

            // Displacement integration uses selectedCourseHeading (travel heading)
            const float dE = stride * std::sin(selectedCourseHeading);
            const float dN = stride * std::cos(selectedCourseHeading);

            east_ += dE;
            north_ += dN;
            cumulativeDistance_ += stride;
            lastStride_ = stride;
            stepCount_++;
            lastStepTimeNs_ = step.tNs;
            lastStepHeading_ = selectedCourseHeading;
            lastPeakHeading_ = peakHeading;

            if (step.intervalS > 0.0f) {
                stepFrequency_ = 1.0f / step.intervalS;
            }

            // Diagnostic log
            PDR_LOGI("step=%u mode=%s peakH=%.1f° phoneH=%.1f° travelH=%.1f° stride=%.2fm dE=%.3fm dN=%.3fm cumE=%.2fm cumN=%.2fm",
                     stepCount_,
                     (courseMode_ == CourseEstimatorMode::CONSTRAINED_TRAVEL_COURSE ? "CONSTRAINED" : "BASELINE"),
                     peakHeading * 180.0f / 3.14159265f,
                     stepPhoneHeading * 180.0f / 3.14159265f,
                     selectedCourseHeading * 180.0f / 3.14159265f,
                     stride, dE, dN, east_, north_);

            return true;
        }

        // Update cadence decay if walking paused
        if (lastStepTimeNs_ > 0 && (tNs - lastStepTimeNs_) > 2000000000LL) { // 2.0s without steps
            stepFrequency_ = 0.0f;
        }

        return false;
    }

    float computeIntervalHeading(int64_t startTNs, int64_t endTNs, float fallbackHeading) const {
        double sinSum = 0.0;
        double cosSum = 0.0;
        int count = 0;

        for (const auto& s : headingHistory_) {
            if (s.tNs >= startTNs && s.tNs <= endTNs) {
                sinSum += std::sin(s.headingRad);
                cosSum += std::cos(s.headingRad);
                count++;
            }
        }

        if (count == 0) {
            return fallbackHeading;
        }

        const double sinMean = sinSum / count;
        const double cosMean = cosSum / count;

        if (std::abs(sinMean) < 1e-12 && std::abs(cosMean) < 1e-12) {
            return fallbackHeading;
        }

        return static_cast<float>(std::atan2(sinMean, cosMean));
    }

    uint32_t getStepCount() const { return stepCount_; }
    float getEast() const { return east_; }
    float getNorth() const { return north_; }
    float getCumulativeDistance() const { return cumulativeDistance_; }
    float getLastStride() const { return lastStride_; }
    float getStepFrequency() const { return stepFrequency_; }
    int64_t getLastStepTimeNs() const { return lastStepTimeNs_; }
    float getLastStepHeading() const { return lastStepHeading_; }
    float getLastPhoneHeading() const { return lastPhoneHeading_; }
    float getLastTravelHeading() const { return lastTravelHeading_; }
    float getLastPeakHeading() const { return lastPeakHeading_; }

    bool isWalking(int64_t currentNs) const {
        if (lastStepTimeNs_ == 0) return false;
        return (currentNs - lastStepTimeNs_) <= 2000000000LL; // Active if step within last 2.0s
    }

    void reset() {
        stepDetector.reset();
        headingHistory_.clear();
        earthAccelHistory_.clear();
        courseObserver_.reset();
        east_ = 0.0f;
        north_ = 0.0f;
        cumulativeDistance_ = 0.0f;
        lastStride_ = 0.0f;
        stepFrequency_ = 0.0f;
        stepCount_ = 0;
        lastStepTimeNs_ = 0;
        latestTimeNs_ = 0;
        lastStepHeading_ = 0.0f;
        lastPhoneHeading_ = 0.0f;
        lastTravelHeading_ = 0.0f;
        lastPeakHeading_ = 0.0f;
    }

private:
    CourseEstimatorMode courseMode_ = CourseEstimatorMode::CONSTRAINED_TRAVEL_COURSE;
    ConstrainedCourseObserver courseObserver_;
    std::deque<HeadingSample> headingHistory_;
    std::deque<EarthAccelSample> earthAccelHistory_;

    float east_ = 0.0f;
    float north_ = 0.0f;
    float cumulativeDistance_ = 0.0f;
    float lastStride_ = 0.0f;
    float stepFrequency_ = 0.0f;
    uint32_t stepCount_ = 0;
    int64_t lastStepTimeNs_ = 0;
    int64_t latestTimeNs_ = 0;
    float lastStepHeading_ = 0.0f;
    float lastPhoneHeading_ = 0.0f;
    float lastTravelHeading_ = 0.0f;
    float lastPeakHeading_ = 0.0f;
};
