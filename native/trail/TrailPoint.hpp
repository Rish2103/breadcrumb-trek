#pragma once

#include <cstdint>
#include <cmath>

enum class TrailRecordingState : int32_t {
    IDLE = 0,
    RECORDING = 1,
    RETURN_READY = 2
};

/**
 * Ordered breadcrumb node per gpt_technical_final.md §16.
 *
 * Each accepted PDR step produces an ordered TrailPoint maintaining
 * chronological sequence ordering P0 → P1 → ... → Pn.
 */
struct TrailPoint {
    uint32_t sequenceIndex = 0;      // Monotonic sequence index 0, 1, 2, ...
    int64_t  timestampNs = 0;        // Hardware monotonic timestamp
    float    east = 0.0f;            // Cartesian East displacement (meters)
    float    north = 0.0f;           // Cartesian North displacement (meters)
    float    heading = 0.0f;         // Compass bearing at step (rad, clockwise from North)
    uint32_t stepCount = 0;          // Total steps up to this breadcrumb
    float    cumulativeDistance = 0.0f; // Cumulative walked distance (meters)
    float    cMag = 0.0f;            // Magnetic reliability at time of point
    float    travelHeading = 0.0f;   // Pedestrian travel direction from ConstrainedCourseObserver (rad)

    /**
     * Rejects NaN, infinite, or non-positive timestamps per §16 validity constraints.
     */
    bool isValid() const {
        if (std::isnan(east) || std::isinf(east)) return false;
        if (std::isnan(north) || std::isinf(north)) return false;
        if (std::isnan(heading) || std::isinf(heading)) return false;
        if (std::isnan(travelHeading) || std::isinf(travelHeading)) return false;
        if (std::isnan(cumulativeDistance) || std::isinf(cumulativeDistance)) return false;
        if (timestampNs <= 0) return false;
        return true;
    }
};
