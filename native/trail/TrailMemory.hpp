#pragma once

#include "TrailPoint.hpp"
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

/**
 * In-memory ordered breadcrumb trail storage per gpt_technical_final.md §16.
 *
 * Preserves strict chronological sequence: P0 → P1 → P2 → ... → Pn.
 * Manages journey lifecycle: IDLE → RECORDING → RETURN_READY.
 *
 * Self-crossing and switchback paths are retained with unique sequence indices.
 * Does not implement aggressive spatial simplification by default in M4.
 */
class TrailMemory {
public:
    TrailMemory() = default;

    /**
     * Starts a new recording journey.
     * Explicitly resets any existing trail and establishes initial breadcrumb P0 at origin (0, 0).
     */
    void startRecording(int64_t tNs, float initialHeadingRad, float initialTravelHeadingRad = 0.0f) {
        reset();
        state_ = TrailRecordingState::RECORDING;
        startTimeNs_ = tNs;
        latestTimeNs_ = tNs;
        endTimeNs_ = tNs;

        // Establish origin anchor breadcrumb P0 (sequence index 0)
        TrailPoint p0;
        p0.sequenceIndex = 0;
        p0.timestampNs = tNs;
        p0.east = 0.0f;
        p0.north = 0.0f;
        p0.heading = initialHeadingRad;
        p0.travelHeading = (initialTravelHeadingRad != 0.0f) ? initialTravelHeadingRad : initialHeadingRad;
        p0.stepCount = 0;
        p0.cumulativeDistance = 0.0f;
        p0.cMag = 1.0f;

        points_.push_back(p0);
    }

    /**
     * Finalizes recording journey, freezing the ordered trail for RETURN navigation.
     */
    void stopRecording(int64_t tNs) {
        if (state_ == TrailRecordingState::RECORDING) {
            state_ = TrailRecordingState::RETURN_READY;
            endTimeNs_ = (tNs > latestTimeNs_) ? tNs : latestTimeNs_;
        }
    }

    /**
     * Resets journey state and frees stored breadcrumb trail.
     */
    void reset() {
        points_.clear();
        state_ = TrailRecordingState::IDLE;
        startTimeNs_ = 0;
        latestTimeNs_ = 0;
        endTimeNs_ = 0;
    }

    /**
     * Appends a new ordered breadcrumb to the trail.
     *
     * Enforces:
     * 1. Must be in RECORDING state.
     * 2. Validity (no NaN, non-infinite, positive timestamp).
     * 3. Monotonic timestamp ordering (tNs > lastPoint.timestampNs).
     * 4. Deduplication against duplicate step events.
     *
     * @return True if point was accepted and appended, false if rejected.
     */
    bool addPoint(int64_t tNs, float east, float north, float heading,
                  uint32_t stepCount, float cumDist, float cMag = 1.0f,
                  float travelHeading = 0.0f) {
        if (state_ != TrailRecordingState::RECORDING) {
            return false;
        }

        TrailPoint pt;
        pt.sequenceIndex = static_cast<uint32_t>(points_.size());
        pt.timestampNs = tNs;
        pt.east = east;
        pt.north = north;
        pt.heading = heading;
        pt.travelHeading = (travelHeading != 0.0f) ? travelHeading : heading;
        pt.stepCount = stepCount;
        pt.cumulativeDistance = cumDist;
        pt.cMag = cMag;

        // Validation rule 1: Mathematical validity
        if (!pt.isValid()) {
            return false;
        }

        if (!points_.empty()) {
            const TrailPoint& last = points_.back();

            // Validation rule 2: Monotonic timestamp ordering
            if (tNs <= last.timestampNs) {
                return false;
            }

            // Validation rule 3: Reject duplicate points
            if (stepCount == last.stepCount &&
                std::abs(east - last.east) < 1e-4f &&
                std::abs(north - last.north) < 1e-4f) {
                return false;
            }
        }

        points_.push_back(pt);
        latestTimeNs_ = tNs;
        endTimeNs_ = tNs;
        return true;
    }

    TrailRecordingState getState() const { return state_; }
    size_t getPointCount() const { return points_.size(); }

    float getTotalDistance() const {
        if (points_.empty()) return 0.0f;
        return points_.back().cumulativeDistance;
    }

    double getJourneyDurationSec(int64_t currentNs = 0) const {
        if (points_.empty() || startTimeNs_ <= 0) return 0.0;
        int64_t end = (state_ == TrailRecordingState::RECORDING)
                          ? (currentNs > latestTimeNs_ ? currentNs : latestTimeNs_)
                          : endTimeNs_;
        return std::max(0.0, static_cast<double>(end - startTimeNs_) * 1e-9);
    }

    const TrailPoint* getStartPoint() const {
        return points_.empty() ? nullptr : &points_.front();
    }

    const TrailPoint* getLatestPoint() const {
        return points_.empty() ? nullptr : &points_.back();
    }

    const TrailPoint* getPoint(size_t index) const {
        if (index >= points_.size()) return nullptr;
        return &points_[index];
    }

    const std::vector<TrailPoint>& getPoints() const { return points_; }

    /**
     * Packs up to maxPoints breadcrumbs into outFloats buffer.
     * Each breadcrumb is packed into 4 floats: [east, north, heading, cumulativeDistance].
     *
     * @return Number of points packed.
     */
    int copyPointsToBuffer(float* outFloats, int maxPoints) const {
        if (outFloats == nullptr || maxPoints <= 0) return 0;
        const int count = std::min(static_cast<int>(points_.size()), maxPoints);
        for (int i = 0; i < count; ++i) {
            const auto& p = points_[i];
            outFloats[i * 4 + 0] = p.east;
            outFloats[i * 4 + 1] = p.north;
            outFloats[i * 4 + 2] = p.heading;
            outFloats[i * 4 + 3] = p.cumulativeDistance;
        }
        return count;
    }

private:
    std::vector<TrailPoint> points_;
    TrailRecordingState state_ = TrailRecordingState::IDLE;
    int64_t startTimeNs_ = 0;
    int64_t latestTimeNs_ = 0;
    int64_t endTimeNs_ = 0;
};
