package com.iqoo.breadcrumb.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CopyOnWriteArrayList

/**
 * Manages discovery, registration, timestamp verification, staleness monitoring,
 * and rate reporting for onboard smartphone sensors.
 *
 * Implements Section 7 (Sensing Layer) of gpt_technical_final.md.
 */
class SensorIngestionManager(context: Context) : SensorEventListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    // Target sensors
    private val accelSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val magSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val pressureSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE)

    // Timestamp & staleness tracking per sensor type
    private val lastTimestamps = ConcurrentHashMap<Int, Long>()
    private val eventCounts = ConcurrentHashMap<Int, Int>()
    private val sensorRatesHz = ConcurrentHashMap<Int, Float>()
    private val healthStates = ConcurrentHashMap<Int, SensorHealthState>()
    private val accuracyStates = ConcurrentHashMap<Int, Int>()

    private var lastRateCalcTimeMs: Long = System.currentTimeMillis()

    // Listeners for dispatched raw events and metrics
    private val eventListeners = CopyOnWriteArrayList<(RawSensorEvent) -> Unit>()
    private val metricListeners = CopyOnWriteArrayList<(Map<Int, Float>, Map<Int, SensorHealthState>) -> Unit>()

    @Volatile
    var isListening: Boolean = false
        private set

    init {
        // Initialize baseline health states
        healthStates[Sensor.TYPE_ACCELEROMETER] = if (accelSensor != null) SensorHealthState.GOOD else SensorHealthState.UNAVAILABLE
        healthStates[Sensor.TYPE_GYROSCOPE] = if (gyroSensor != null) SensorHealthState.GOOD else SensorHealthState.UNAVAILABLE
        healthStates[Sensor.TYPE_MAGNETIC_FIELD] = if (magSensor != null) SensorHealthState.GOOD else SensorHealthState.UNAVAILABLE
        healthStates[Sensor.TYPE_PRESSURE] = if (pressureSensor != null) SensorHealthState.GOOD else SensorHealthState.UNAVAILABLE
    }

    /**
     * Discovers all supported navigation sensors and returns hardware capabilities.
     */
    fun discoverSensors(): List<SensorCapability> {
        val targets = listOf(
            Triple(Sensor.TYPE_ACCELEROMETER, "Accelerometer", accelSensor),
            Triple(Sensor.TYPE_GYROSCOPE, "Gyroscope", gyroSensor),
            Triple(Sensor.TYPE_MAGNETIC_FIELD, "Magnetometer", magSensor),
            Triple(Sensor.TYPE_PRESSURE, "Barometer", pressureSensor)
        )

        return targets.map { (type, typeName, sensor) ->
            if (sensor != null) {
                SensorCapability(
                    type = type,
                    typeName = typeName,
                    name = sensor.name,
                    vendor = sensor.vendor,
                    version = sensor.version,
                    minDelayUs = sensor.minDelay,
                    maxDelayUs = sensor.maxDelay,
                    resolution = sensor.resolution,
                    powerMa = sensor.power,
                    isAvailable = true
                )
            } else {
                SensorCapability(
                    type = type,
                    typeName = typeName,
                    name = "Not Available",
                    vendor = "N/A",
                    version = 0,
                    minDelayUs = 0,
                    maxDelayUs = 0,
                    resolution = 0.0f,
                    powerMa = 0.0f,
                    isAvailable = false
                )
            }
        }
    }

    /**
     * Registers listeners with requested rates:
     * - Accel: 20_000 µs (50 Hz)
     * - Gyro: 20_000 µs (50 Hz)
     * - Mag: 50_000 µs (20 Hz)
     * - Baro: 100_000 µs (10 Hz, if present)
     */
    @Synchronized
    fun startListening() {
        if (isListening) return

        lastTimestamps.clear()
        eventCounts.clear()
        sensorRatesHz.clear()
        lastRateCalcTimeMs = System.currentTimeMillis()

        accelSensor?.let { sensorManager.registerListener(this, it, 20_000) }
        gyroSensor?.let { sensorManager.registerListener(this, it, 20_000) }
        magSensor?.let { sensorManager.registerListener(this, it, 50_000) }
        pressureSensor?.let { sensorManager.registerListener(this, it, 100_000) }

        isListening = true
    }

    /**
     * Unregisters all sensor listeners.
     */
    @Synchronized
    fun stopListening() {
        if (!isListening) return
        sensorManager.unregisterListener(this)
        isListening = false
    }

    fun addEventListener(listener: (RawSensorEvent) -> Unit) {
        eventListeners.add(listener)
    }

    fun removeEventListener(listener: (RawSensorEvent) -> Unit) {
        eventListeners.remove(listener)
    }

    fun addMetricListener(listener: (rates: Map<Int, Float>, health: Map<Int, SensorHealthState>) -> Unit) {
        metricListeners.add(listener)
    }

    fun removeMetricListener(listener: (rates: Map<Int, Float>, health: Map<Int, SensorHealthState>) -> Unit) {
        metricListeners.remove(listener)
    }

    override fun onSensorChanged(event: SensorEvent) {
        val type = event.sensor.type
        val tNs = event.timestamp

        // Monotonic timestamp check (reject non-monotonic samples per §7.3)
        val lastT = lastTimestamps[type]
        if (lastT != null) {
            val dtNs = tNs - lastT
            if (dtNs <= 0L) {
                // Reject non-monotonic sample
                return
            }
            val dtSeconds = dtNs * 1e-9f
            if (dtSeconds > 0.25f) {
                // Large timestamp jump guard; update timestamp but flag degraded stream
                healthStates[type] = SensorHealthState.DEGRADED
            }
        }
        lastTimestamps[type] = tNs
        eventCounts[type] = (eventCounts[type] ?: 0) + 1

        val (x, y, z) = when (type) {
            Sensor.TYPE_PRESSURE -> Triple(event.values[0], 0.0f, 0.0f)
            else -> Triple(event.values[0], event.values[1], event.values[2])
        }

        val rawEvent = RawSensorEvent(
            timestampNs = tNs,
            sensorType = type,
            x = x,
            y = y,
            z = z,
            accuracy = event.accuracy
        )

        // Dispatch to subscribers
        for (i in 0 until eventListeners.size) {
            eventListeners[i](rawEvent)
        }

        checkPeriodicMetrics(tNs)
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
        accuracyStates[sensor.type] = accuracy
        if (accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) {
            healthStates[sensor.type] = SensorHealthState.DEGRADED
        } else if (healthStates[sensor.type] == SensorHealthState.DEGRADED) {
            healthStates[sensor.type] = SensorHealthState.GOOD
        }
    }

    /**
     * Checks rate calculations and stream staleness once every ~1000 ms.
     */
    private fun checkPeriodicMetrics(currentEventTimestampNs: Long) {
        val nowMs = System.currentTimeMillis()
        if (nowMs - lastRateCalcTimeMs >= 1000L) {
            val elapsedSec = (nowMs - lastRateCalcTimeMs) / 1000.0f
            lastRateCalcTimeMs = nowMs

            for ((type, count) in eventCounts) {
                val hz = count / elapsedSec
                sensorRatesHz[type] = hz
                eventCounts[type] = 0
            }

            // Staleness evaluation per §7.3
            // Accel > 100 ms is stale
            val lastAccel = lastTimestamps[Sensor.TYPE_ACCELEROMETER]
            if (accelSensor != null) {
                val accelStale = (lastAccel == null) || ((currentEventTimestampNs - lastAccel) > 100_000_000L)
                healthStates[Sensor.TYPE_ACCELEROMETER] = if (accelStale) SensorHealthState.STALE else SensorHealthState.GOOD
            }

            // Mag > 300 ms is stale
            val lastMag = lastTimestamps[Sensor.TYPE_MAGNETIC_FIELD]
            if (magSensor != null) {
                val magStale = (lastMag == null) || ((currentEventTimestampNs - lastMag) > 300_000_000L)
                healthStates[Sensor.TYPE_MAGNETIC_FIELD] = if (magStale) SensorHealthState.STALE else SensorHealthState.GOOD
            }

            // Gyro > 100 ms is stale
            val lastGyro = lastTimestamps[Sensor.TYPE_GYROSCOPE]
            if (gyroSensor != null) {
                val gyroStale = (lastGyro == null) || ((currentEventTimestampNs - lastGyro) > 100_000_000L)
                healthStates[Sensor.TYPE_GYROSCOPE] = if (gyroStale) SensorHealthState.STALE else SensorHealthState.GOOD
            }

            // Dispatch metrics
            val ratesSnapshot = HashMap(sensorRatesHz)
            val healthSnapshot = HashMap(healthStates)
            for (i in 0 until metricListeners.size) {
                metricListeners[i](ratesSnapshot, healthSnapshot)
            }
        }
    }

    /**
     * Diagnostic snapshot for verification and testing.
     */
    fun getStreamSnapshot(): Map<Int, StreamStatus> {
        val snapshot = mutableMapOf<Int, StreamStatus>()
        val types = listOf(
            Sensor.TYPE_ACCELEROMETER,
            Sensor.TYPE_GYROSCOPE,
            Sensor.TYPE_MAGNETIC_FIELD,
            Sensor.TYPE_PRESSURE
        )
        for (t in types) {
            snapshot[t] = StreamStatus(
                rateHz = sensorRatesHz[t] ?: 0.0f,
                health = healthStates[t] ?: SensorHealthState.UNAVAILABLE,
                lastTimestampNs = lastTimestamps[t] ?: 0L
            )
        }
        return snapshot
    }

    data class StreamStatus(
        val rateHz: Float,
        val health: SensorHealthState,
        val lastTimestampNs: Long
    )
}
