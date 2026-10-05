package com.iqoo.breadcrumb.sensors

/**
 * Raw sensor sample used across live ingestion and replay streams.
 *
 * Coordinates and units adhere strictly to gpt_technical_final.md:
 * - Accelerometer: m/s²
 * - Gyroscope: rad/s
 * - Magnetometer: µT
 * - Barometer: hPa (stored in x, y = 0.0, z = 0.0)
 * - Timestamp: nanoseconds from SensorEvent.timestamp
 */
data class RawSensorEvent(
    val timestampNs: Long,
    val sensorType: Int,
    val x: Float,
    val y: Float,
    val z: Float,
    val accuracy: Int = 0
) {
    /**
     * Serializes this event into standard replay-compatible CSV line.
     * Schema: timestamp_ns,sensor_type,x,y,z
     */
    fun toCsvLine(): String = "$timestampNs,$sensorType,$x,$y,$z"

    companion object {
        /**
         * Parses a CSV row adhering to the replay schema.
         */
        fun fromCsvLine(line: String): RawSensorEvent? {
            val tokens = line.split(',')
            if (tokens.size < 5) return null
            return try {
                RawSensorEvent(
                    timestampNs = tokens[0].trim().toLong(),
                    sensorType = tokens[1].trim().toInt(),
                    x = tokens[2].trim().toFloat(),
                    y = tokens[3].trim().toFloat(),
                    z = tokens[4].trim().toFloat()
                )
            } catch (_: NumberFormatException) {
                null
            }
        }
    }
}
