package com.iqoo.breadcrumb.ui

import android.annotation.SuppressLint
import android.hardware.Sensor
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.iqoo.breadcrumb.NavigationNative
import com.iqoo.breadcrumb.NativeNavState
import com.iqoo.breadcrumb.R
import com.iqoo.breadcrumb.sensors.CsvSensorRecorder
import com.iqoo.breadcrumb.sensors.RawSensorEvent
import com.iqoo.breadcrumb.sensors.SensorHealthState
import com.iqoo.breadcrumb.sensors.SensorIngestionManager
import java.io.File

/**
 * MainActivity for Milestone 4:
 * - Pushes sensor streams into native C++ AHRS and PDR engine over JNI
 * - Retrieves native orientation, PDR position, and ordered breadcrumb trail
 * - Manages trail recording lifecycle (IDLE -> RECORDING -> RETURN_READY)
 * - Renders relative 2D local breadcrumb path on TrailCanvasView without map/GPS
 * - Displays live metrics and logs to BreadcrumbPDR and BreadcrumbAHRS
 */
class MainActivity : AppCompatActivity() {

    private lateinit var sensorManager: SensorIngestionManager
    private val recorder = CsvSensorRecorder()

    // Preallocated 40-float native state buffer (zero per-poll allocation)
    private val nativeState = FloatArray(40)
    // Preallocated buffer for up to 500 trail points (4 floats per point: E, N, heading, dist)
    private val trailBuffer = FloatArray(2000)

    private lateinit var textStatus: TextView
    private lateinit var textJniStatus: TextView
    private lateinit var textAhrsState: TextView
    private lateinit var textPdrState: TextView
    private lateinit var textTrailMetrics: TextView
    private lateinit var textGuidanceMetrics: TextView
    private lateinit var trailCanvasView: TrailCanvasView
    private lateinit var textSensorDiscovery: TextView
    private lateinit var textSensorRates: TextView
    private lateinit var textRecordStatus: TextView
    private lateinit var btnStartRecord: Button
    private lateinit var btnStopRecord: Button
    private lateinit var btnStartTrail: Button
    private lateinit var btnStopTrail: Button
    private lateinit var btnResetTrail: Button
    private lateinit var btnStartReturn: Button
    private lateinit var btnStopReturn: Button

    private val uiHandler = Handler(Looper.getMainLooper())
    private var currentRates = mapOf<Int, Float>()
    private var currentHealth = mapOf<Int, SensorHealthState>()

    @SuppressLint("SetTextI18n")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        textStatus = findViewById(R.id.text_status)
        textJniStatus = findViewById(R.id.text_jni_status)
        textAhrsState = findViewById(R.id.text_ahrs_state)
        textPdrState = findViewById(R.id.text_pdr_state)
        textTrailMetrics = findViewById(R.id.text_trail_metrics)
        textGuidanceMetrics = findViewById(R.id.text_guidance_metrics)
        trailCanvasView = findViewById(R.id.trail_canvas_view)
        textSensorDiscovery = findViewById(R.id.text_sensor_discovery)
        textSensorRates = findViewById(R.id.text_sensor_rates)
        textRecordStatus = findViewById(R.id.text_record_status)
        btnStartRecord = findViewById(R.id.btn_start_record)
        btnStopRecord = findViewById(R.id.btn_stop_record)
        btnStartTrail = findViewById(R.id.btn_start_trail)
        btnStopTrail = findViewById(R.id.btn_stop_trail)
        btnResetTrail = findViewById(R.id.btn_reset_trail)
        btnStartReturn = findViewById(R.id.btn_start_return)
        btnStopReturn = findViewById(R.id.btn_stop_return)

        textStatus.text = getString(R.string.status_ready)

        // 1. JNI Sanity Verification
        try {
            val version = NavigationNative.nativeGetVersion()
            textJniStatus.text = "Native Engine Version: $version (JNI Linkage OK)"
        } catch (t: Throwable) {
            textJniStatus.text = "Native Engine Error: ${t.message}"
        }

        // 2. Sensor Ingestion & Native Pipeline
        sensorManager = SensorIngestionManager(this)

        val capabilities = sensorManager.discoverSensors()
        val discoverySb = StringBuilder()
        for (cap in capabilities) {
            discoverySb.append("• ${cap.typeName}: ")
            if (cap.isAvailable) {
                discoverySb.append("${cap.name} [Vendor: ${cap.vendor}, MinDelay: ${cap.minDelayUs}µs, Res: ${cap.resolution}]\n")
            } else {
                discoverySb.append("NOT AVAILABLE (Hardware Absent)\n")
            }
        }
        textSensorDiscovery.text = discoverySb.toString().trimEnd()

        sensorManager.addEventListener { rawEvent ->
            dispatchToNative(rawEvent)
            if (recorder.isRecordingActive()) {
                recorder.recordEvent(rawEvent)
            }
        }

        sensorManager.addMetricListener { rates, health ->
            currentRates = rates
            currentHealth = health
        }

        // 3. Trail Controls
        btnStartTrail.setOnClickListener {
            NavigationNative.startTrailRecording()
        }

        btnStopTrail.setOnClickListener {
            NavigationNative.stopTrailRecording()
        }

        btnResetTrail.setOnClickListener {
            NavigationNative.resetTrail()
        }

        btnStartReturn.setOnClickListener {
            NavigationNative.startReverseNavigation()
        }

        btnStopReturn.setOnClickListener {
            NavigationNative.stopReverseNavigation()
        }

        // 4. CSV Recorder Controls
        val recordDir = File(getExternalFilesDir(null) ?: filesDir, "sensor_recordings")
        btnStartRecord.setOnClickListener {
            val file = recorder.startRecording(recordDir, "walk_recording")
            btnStartRecord.isEnabled = false
            btnStopRecord.isEnabled = true
            textRecordStatus.text = "RECORDING ACTIVE\nFile: ${file.name}\nSamples: 0"
        }

        btnStopRecord.setOnClickListener {
            val summary = recorder.stopRecording()
            btnStartRecord.isEnabled = true
            btnStopRecord.isEnabled = false
            if (summary != null) {
                textRecordStatus.text = "RECORDING STOPPED\nFile: ${summary.file.name}\nSamples: ${summary.sampleCount}\nDuration: ${summary.durationMs / 1000.0}s\nSize: ${summary.fileSizeBytes} bytes\nPath: ${summary.file.absolutePath}"
            } else {
                textRecordStatus.text = "Recorder stopped."
            }
        }

        handleIntent(intent)
    }

    override fun onNewIntent(intent: android.content.Intent?) {
        super.onNewIntent(intent)
        setIntent(intent)
        handleIntent(intent)
    }

    private fun handleIntent(intent: android.content.Intent?) {
        if (intent == null) return
        val variant = intent.getStringExtra("variant")
        if (variant != null) {
            if (variant.equals("B", ignoreCase = true) || variant == "1") {
                NavigationNative.setMadgwickVariant(1)
            } else {
                NavigationNative.setMadgwickVariant(0)
            }
        }

        val action = intent.getStringExtra("action") ?: return
        val recordDir = File(getExternalFilesDir(null) ?: filesDir, "sensor_recordings")

        when (action) {
            "start_record" -> {
                if (!recorder.isRecordingActive()) {
                    val file = recorder.startRecording(recordDir, "walk_recording")
                    btnStartRecord.isEnabled = false
                    btnStopRecord.isEnabled = true
                    textRecordStatus.text = "RECORDING ACTIVE\nFile: ${file.name}\nSamples: 0"
                }
            }
            "stop_record" -> {
                if (recorder.isRecordingActive()) {
                    val summary = recorder.stopRecording()
                    btnStartRecord.isEnabled = true
                    btnStopRecord.isEnabled = false
                    if (summary != null) {
                        textRecordStatus.text = "RECORDING STOPPED\nFile: ${summary.file.name}\nSamples: ${summary.sampleCount}\nDuration: ${summary.durationMs / 1000.0}s\nSize: ${summary.fileSizeBytes} bytes\nPath: ${summary.file.absolutePath}"
                    }
                }
            }
            "start_trail" -> NavigationNative.startTrailRecording()
            "stop_trail" -> NavigationNative.stopTrailRecording()
            "reset_trail" -> NavigationNative.resetTrail()
            "start_return" -> NavigationNative.startReverseNavigation()
            "stop_return" -> NavigationNative.stopReverseNavigation()
            "set_variant_a" -> NavigationNative.setMadgwickVariant(0)
            "set_variant_b" -> NavigationNative.setMadgwickVariant(1)
            "inject_test_trail" -> {
                NavigationNative.startTrailRecording()
                val t0 = System.nanoTime()
                for (i in 1..10) {
                    NavigationNative.pushAccel(0.0f, 9.8f, 3.8f, t0 + i * 500_000_000L)
                    NavigationNative.pushAccel(0.0f, 9.8f, -2.8f, t0 + i * 500_000_000L + 250_000_000L)
                }
                NavigationNative.stopTrailRecording()
            }
        }
    }

    private fun dispatchToNative(event: RawSensorEvent) {
        when (event.sensorType) {
            Sensor.TYPE_ACCELEROMETER -> {
                NavigationNative.pushAccel(event.x, event.y, event.z, event.timestampNs)
            }
            Sensor.TYPE_GYROSCOPE -> {
                NavigationNative.pushGyro(event.x, event.y, event.z, event.timestampNs)
            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                NavigationNative.pushMag(event.x, event.y, event.z, event.timestampNs)
            }
        }
    }

    override fun onResume() {
        super.onResume()
        sensorManager.startListening()
        startUiLoop()
    }

    override fun onPause() {
        super.onPause()
        stopUiLoop()
        if (!recorder.isRecordingActive()) {
            sensorManager.stopListening()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        recorder.stopRecording()
        sensorManager.stopListening()
        stopUiLoop()
    }

    private var lastLogTimeMs = 0L

    /**
     * 4 Hz UI update loop: extracts native state and updates UI without per-sample allocations.
     */
    private val uiLoopRunnable = object : Runnable {
        @SuppressLint("SetTextI18n")
        override fun run() {
            val count = NavigationNative.nativeGetState(nativeState)
            if (count >= 16) {
                val navState = NativeNavState.fromBuffer(nativeState)

                val statStr = if (navState.isStationary) "STATIONARY" else "MOVING"
                val magBaseStr = if (navState.calibratedB0 > 0.0f) {
                    "%.1f µT (CALIBRATED)".format(navState.calibratedB0)
                } else {
                    "UNAVAILABLE (Calibrating during stationary hold)"
                }

                val variantStr = if (NavigationNative.getMadgwickVariant() == 1) "VARIANT B (Yaw-Decoupled 6-DOF)" else "VARIANT A (Production)"

                // 1. AHRS Dashboard
                textAhrsState.text = """
                    AHRS Mode: %s
                    Heading: %.1f° (%.3f rad)
                    Attitude q: [%.3f, %.3f, %.3f, %.3f]
                    Mag Reliability (Cmag): %.2f
                    Local Mag Baseline: %s
                    Stationary State: %s
                    Gyro Bias Norm: %.4f rad/s
                """.trimIndent().format(
                    variantStr,
                    navState.headingDeg, navState.headingRad,
                    navState.qw, navState.qx, navState.qy, navState.qz,
                    navState.cMag, magBaseStr, statStr, navState.gyroBiasNorm
                )

                // 2. PDR Dashboard
                val motionStateStr = if (navState.isStationary) {
                    "STATIONARY"
                } else if (navState.stepFrequency > 0.0f) {
                    "WALKING (%.2f Hz)".format(navState.stepFrequency)
                } else {
                    "MOVING"
                }

                textPdrState.text = """
                    Steps: %d
                    Distance: %.2f m
                    Stride: %.2f m
                    Heading: %.1f°
                    State: %s
                    Position: E %.2f m, N %.2f m
                """.trimIndent().format(
                    navState.stepCount,
                    navState.cumulativeDistance,
                    navState.strideLength,
                    navState.headingDeg,
                    motionStateStr,
                    navState.east, navState.north
                )

                // 3. Ordered Breadcrumb Trail & Canvas Visualizer
                val trailStateStr = when (navState.trailStateCode) {
                    1 -> "RECORDING"
                    2 -> "RETURN_READY"
                    else -> "IDLE"
                }

                val ptsCopied = NavigationNative.getTrailPoints(trailBuffer, 500)
                trailCanvasView.updateTrail(
                    rawPoints = trailBuffer,
                    count = ptsCopied,
                    curE = navState.east,
                    curN = navState.north,
                    curHeading = navState.headingRad,
                    stateStr = trailStateStr,
                    isRevActive = navState.isReverseNavActive,
                    cE = navState.carrotEast,
                    cN = navState.carrotNorth,
                    pE = navState.projEast,
                    pN = navState.projNorth,
                    xte = navState.crossTrackError,
                    nState = navState.navStateName,
                    gMode = navState.guidanceModeName,
                    tCmd = navState.turnCommandName,
                    recovAct = navState.recoveryActive,
                    gE = navState.guidanceVectorEast,
                    gN = navState.guidanceVectorNorth
                )

                textTrailMetrics.text = "Trail: $trailStateStr | Points: ${navState.trailPointCount} | Distance: ${"%.2f".format(navState.totalTrailDistance)}m | Duration: ${"%.1f".format(navState.journeyDurationSec)}s"

                if (navState.isReverseNavActive) {
                    textGuidanceMetrics.text = "RETURN: ${navState.navStateName} [${navState.guidanceModeName}] | CMD: ${navState.turnCommandName} | XTE: ${"%.2f".format(navState.crossTrackError)}m | Steer: ${"%.1f".format(navState.steeringErrorDeg)}° | Seg: ${navState.matchedSegmentIndex} | Complete: ${navState.isReturnComplete}"
                } else {
                    textGuidanceMetrics.text = if (navState.isReturnReady) "Return Nav: READY TO START" else "Return Nav: INACTIVE"
                }

                btnStartTrail.isEnabled = !navState.isTrailRecording && !navState.isReverseNavActive
                btnStopTrail.isEnabled = navState.isTrailRecording
                btnResetTrail.isEnabled = !navState.isTrailRecording && !navState.isReverseNavActive
                btnStartReturn.isEnabled = navState.isReturnReady && !navState.isReverseNavActive
                btnStopReturn.isEnabled = navState.isReverseNavActive

                if (System.currentTimeMillis() - lastLogTimeMs >= 1000L) {
                    lastLogTimeMs = System.currentTimeMillis()
                    android.util.Log.i(
                        "BreadcrumbPDR",
                        "Steps: ${navState.stepCount} | Dist: ${"%.2f".format(navState.cumulativeDistance)}m | TrailPts: ${navState.trailPointCount} | TrailState: $trailStateStr | Pos: (E=${"%.2f".format(navState.east)}, N=${"%.2f".format(navState.north)}) | Heading: ${"%.1f".format(navState.headingDeg)}° | State: $motionStateStr"
                    )
                    if (navState.isReverseNavActive) {
                        android.util.Log.i(
                            "BreadcrumbNav",
                            "Return: ${navState.navStateName} [${navState.guidanceModeName}] | CMD: ${navState.turnCommandName} | Seg: ${navState.matchedSegmentIndex} | XTE: ${"%.2f".format(navState.crossTrackError)}m | Steer: ${"%.1f".format(navState.steeringErrorDeg)}° | DesiredHead: ${"%.1f".format(navState.desiredHeadingDeg)}° | Carrot: (${"%.2f".format(navState.carrotEast)}, ${"%.2f".format(navState.carrotNorth)}) | RecovAct: ${navState.recoveryActive} | Complete: ${navState.isReturnComplete}"
                        )
                    }
                }
            }

            // Update streaming rates
            val accelRate = currentRates[Sensor.TYPE_ACCELEROMETER] ?: 0.0f
            val accelHealth = currentHealth[Sensor.TYPE_ACCELEROMETER] ?: SensorHealthState.UNAVAILABLE
            val gyroRate = currentRates[Sensor.TYPE_GYROSCOPE] ?: 0.0f
            val gyroHealth = currentHealth[Sensor.TYPE_GYROSCOPE] ?: SensorHealthState.UNAVAILABLE
            val magRate = currentRates[Sensor.TYPE_MAGNETIC_FIELD] ?: 0.0f
            val magHealth = currentHealth[Sensor.TYPE_MAGNETIC_FIELD] ?: SensorHealthState.UNAVAILABLE
            val baroRate = currentRates[Sensor.TYPE_PRESSURE] ?: 0.0f
            val baroHealth = currentHealth[Sensor.TYPE_PRESSURE] ?: SensorHealthState.UNAVAILABLE

            textSensorRates.text = """
                Accel: %.1f Hz [%s]
                Gyro:  %.1f Hz [%s]
                Mag:   %.1f Hz [%s]
                Baro:  %.1f Hz [%s]
            """.trimIndent().format(
                accelRate, accelHealth.name,
                gyroRate, gyroHealth.name,
                magRate, magHealth.name,
                baroRate, baroHealth.name
            )

            uiHandler.postDelayed(this, 250) // 4 Hz
        }
    }

    private fun startUiLoop() {
        uiHandler.removeCallbacks(uiLoopRunnable)
        uiHandler.post(uiLoopRunnable)
    }

    private fun stopUiLoop() {
        uiHandler.removeCallbacks(uiLoopRunnable)
    }
}
