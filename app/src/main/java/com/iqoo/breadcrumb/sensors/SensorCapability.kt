package com.iqoo.breadcrumb.sensors

/**
 * Metadata and hardware capabilities of a discovered sensor.
 */
data class SensorCapability(
    val type: Int,
    val typeName: String,
    val name: String,
    val vendor: String,
    val version: Int,
    val minDelayUs: Int,
    val maxDelayUs: Int,
    val resolution: Float,
    val powerMa: Float,
    val isAvailable: Boolean
)
