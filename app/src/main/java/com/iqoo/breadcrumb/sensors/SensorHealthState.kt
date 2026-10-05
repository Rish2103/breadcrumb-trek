package com.iqoo.breadcrumb.sensors

/**
 * Health and staleness state of an individual sensor stream (per §7.4).
 */
enum class SensorHealthState {
    GOOD,
    DEGRADED,
    STALE,
    UNAVAILABLE
}
