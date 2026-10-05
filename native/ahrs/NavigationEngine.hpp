#pragma once

#include "MathTypes.hpp"
#include "Madgwick9DOF.hpp"
#include "MagneticReliability.hpp"
#include "StationaryDetector.hpp"
#include "../pdr/PdrEngine.hpp"
#include "../trail/TrailMemory.hpp"
#include "../guidance/ReverseNavigator.hpp"
#include <cstdint>

class NavigationEngine {
public:
    static NavigationEngine& instance() {
        static NavigationEngine inst;
        return inst;
    }

    void pushAccel(float x, float y, float z, int64_t tNs);
    void pushGyro(float x, float y, float z, int64_t tNs);
    void pushMag(float x, float y, float z, int64_t tNs);

    int getState(float* outBuffer, int size);
    void reset();

    // Trail controls
    bool startTrailRecording();
    bool stopTrailRecording();
    void resetTrail();
    int  getTrailPoints(float* outFloats, int maxPoints);

    // Reverse navigation controls (M5)
    bool startReverseNavigation();
    void stopReverseNavigation();
    bool isReverseNavigationActive() const;
    breadcrumb::GuidanceOut getGuidanceOutput() const;
    breadcrumb::ReverseNavigator& getReverseNavigator() { return reverseNav_; }
    const breadcrumb::ReverseNavigator& getReverseNavigator() const { return reverseNav_; }

    // Accessors for native unit tests
    Quaternion getQuaternion() const { return ahrs_.q; }
    double getHeading() const { return phoneForwardBearing(ahrs_.q); }
    float getCmag() const { return magRel_.getCmag(); }
    bool isStationary() const { return statDet_.isStationary(); }
    Vec3 getGyroBias() const { return statDet_.gyroBias; }
    bool isMagCalibrated() const { return magRel_.isCalibrated(); }
    float getCalibratedB0() const { return magRel_.getB0(); }
    float getCalibratedDip0() const { return magRel_.getDip0(); }
    void setMagReference(float b0, float dip0) { magRel_.setReference(b0, dip0); }
    void setMadgwickVariant(int variant) { ahrs_.decoupleYawAccelGradient = (variant == 1); }
    int getMadgwickVariant() const { return ahrs_.decoupleYawAccelGradient ? 1 : 0; }

    // PDR accessors
    uint32_t getStepCount() const { return pdr_.getStepCount(); }
    float getEast() const { return pdr_.getEast(); }
    float getNorth() const { return pdr_.getNorth(); }
    float getCumulativeDistance() const { return pdr_.getCumulativeDistance(); }
    float getLastStride() const { return pdr_.getLastStride(); }
    float getStepFrequency() const { return pdr_.getStepFrequency(); }
    bool isWalking() const { return pdr_.isWalking(lastAccelT_); }
    PdrEngine& getPdrEngine() { return pdr_; }
    TrailMemory& getTrailMemory() { return trail_; }
    const TrailMemory& getTrailMemory() const { return trail_; }

    // Course estimator mode (M3.8)
    void setCourseMode(CourseEstimatorMode mode) { pdr_.setCourseMode(mode); }
    CourseEstimatorMode getCourseMode() const { return pdr_.getCourseMode(); }
    float getTravelHeading() const { return pdr_.getLastStepHeading(); }

private:
    NavigationEngine() = default;

    Madgwick9DOF ahrs_;
    MagneticReliability magRel_;
    StationaryDetector statDet_;
    PdrEngine pdr_;
    TrailMemory trail_;
    breadcrumb::ReverseNavigator reverseNav_;

    Vec3 latestAccel_{0.0f, 0.0f, 0.0f};
    Vec3 latestGyro_{0.0f, 0.0f, 0.0f};
    Vec3 latestMag_{0.0f, 0.0f, 0.0f};

    int64_t lastAccelT_ = 0;
    int64_t lastGyroT_ = 0;
    int64_t lastMagT_ = 0;

    bool init_ = false;
};
