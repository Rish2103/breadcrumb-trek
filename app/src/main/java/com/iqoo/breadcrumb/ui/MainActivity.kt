package com.iqoo.breadcrumb.ui

import android.annotation.SuppressLint
import android.graphics.Color
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
import com.iqoo.breadcrumb.sensors.RawSensorEvent
import com.iqoo.breadcrumb.sensors.SensorIngestionManager

/**
 * MainActivity for Breadcrumb Trek 2.x
 *
 * Clean single-screen demonstration UI:
 * - Status Card (Current Mode, Navigation State, Confidence)
 * - Live Metrics (Steps, Distance, Travel Heading, Phone Heading, Cross-Track Error)
 * - Main Visual (TrailCanvasView with 2D relative trail, carrot, and recovery vectors)
 * - Controls (Start Recording, Stop Recording, Return, Reset)
 */
class MainActivity : AppCompatActivity() {

    private lateinit var sensorManager: SensorIngestionManager

    // Preallocated 40-float native state buffer (zero per-poll allocation)
    private val nativeState = FloatArray(40)
    // Preallocated buffer for up to 500 trail points (4 floats per point: E, N, heading, dist)
    private val trailBuffer = FloatArray(2000)

    // UI Header & Status Card
    private lateinit var textMode: TextView
    private lateinit var textNavState: TextView
    private lateinit var textConfidence: TextView

    // Live Metrics
    private lateinit var textMetricSteps: TextView
    private lateinit var textMetricDistance: TextView
    private lateinit var textMetricTravelHeading: TextView
    private lateinit var textMetricPhoneHeading: TextView
    private lateinit var textMetricXte: TextView

    // Canvas & Controls
    private lateinit var trailCanvasView: TrailCanvasView
    private lateinit var btnStartRecord: Button
    private lateinit var btnStopRecord: Button
    private lateinit var btnReturn: Button
    private lateinit var btnReset: Button

    private val uiHandler = Handler(Looper.getMainLooper())
    private var lastLogTimeMs = 0L

    @SuppressLint("SetTextI18n")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // 1. Bind Views
        textMode = findViewById(R.id.text_mode)
        textNavState = findViewById(R.id.text_nav_state)
        textConfidence = findViewById(R.id.text_confidence)

        textMetricSteps = findViewById(R.id.text_metric_steps)
        textMetricDistance = findViewById(R.id.text_metric_distance)
        textMetricTravelHeading = findViewById(R.id.text_metric_travel_heading)
        textMetricPhoneHeading = findViewById(R.id.text_metric_phone_heading)
        textMetricXte = findViewById(R.id.text_metric_xte)

        trailCanvasView = findViewById(R.id.trail_canvas_view)
        btnStartRecord = findViewById(R.id.btn_start_record)
        btnStopRecord = findViewById(R.id.btn_stop_record)
        btnReturn = findViewById(R.id.btn_return)
        btnReset = findViewById(R.id.btn_reset)

        // 2. Set Button Listeners
        btnStartRecord.setOnClickListener {
            NavigationNative.startTrailRecording()
        }

        btnStopRecord.setOnClickListener {
            NavigationNative.stopTrailRecording()
        }

        btnReturn.setOnClickListener {
            if (NavigationNative.isReverseNavigationActive()) {
                NavigationNative.stopReverseNavigation()
            } else {
                NavigationNative.startReverseNavigation()
            }
        }

        btnReset.setOnClickListener {
            if (NavigationNative.isReverseNavigationActive()) {
                NavigationNative.stopReverseNavigation()
            }
            NavigationNative.resetTrail()
            NavigationNative.nativeReset()
        }

        // 3. Initialize Sensor Ingestion
        sensorManager = SensorIngestionManager(this)
        sensorManager.addEventListener { event: RawSensorEvent ->
            dispatchToNative(event)
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
        when (action) {
            "start_trail", "start_record" -> NavigationNative.startTrailRecording()
            "stop_trail", "stop_record" -> NavigationNative.stopTrailRecording()
            "reset_trail", "reset" -> {
                if (NavigationNative.isReverseNavigationActive()) {
                    NavigationNative.stopReverseNavigation()
                }
                NavigationNative.resetTrail()
                NavigationNative.nativeReset()
            }
            "start_return", "return" -> NavigationNative.startReverseNavigation()
            "stop_return" -> NavigationNative.stopReverseNavigation()
            "set_variant_a" -> NavigationNative.setMadgwickVariant(0)
            "set_variant_b" -> NavigationNative.setMadgwickVariant(1)
            "demo_walk" -> {
                NavigationNative.startTrailRecording()
                var tNs = System.nanoTime()
                val dtNs = 20_000_000L // 50 Hz
                // 5 walking steps
                for (step in 0 until 5) {
                    for (i in 0 until 25) {
                        val phase = 2.0 * Math.PI * (i / 25.0)
                        val az = (9.81f + 4.0f * Math.sin(phase)).toFloat()
                        val ay = (1.5f * Math.cos(phase)).toFloat()
                        NavigationNative.pushAccel(0.0f, ay, az, tNs)
                        NavigationNative.pushGyro(0.0f, 0.0f, 0.0f, tNs)
                        tNs += dtNs
                    }
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
        sensorManager.stopListening()
    }

    override fun onDestroy() {
        super.onDestroy()
        sensorManager.stopListening()
        stopUiLoop()
    }

    /**
     * 4 Hz UI update loop: extracts native state and updates UI without per-sample allocations.
     */
    private val uiLoopRunnable = object : Runnable {
        @SuppressLint("SetTextI18n")
        override fun run() {
            val count = NavigationNative.nativeGetState(nativeState)
            if (count >= 16) {
                val navState = NativeNavState.fromBuffer(nativeState)

                // 1. Status Card: Mode
                val modeStr = when {
                    navState.isReverseNavActive -> "RETURN ACTIVE"
                    navState.isTrailRecording -> "RECORDING"
                    navState.isReturnReady -> "RETURN READY"
                    else -> "IDLE"
                }
                textMode.text = modeStr
                when (modeStr) {
                    "RETURN ACTIVE" -> textMode.setTextColor(Color.parseColor("#EC4899")) // Magenta
                    "RECORDING" -> textMode.setTextColor(Color.parseColor("#2DD4BF"))     // Teal
                    "RETURN READY" -> textMode.setTextColor(Color.parseColor("#F59E0B"))  // Amber
                    else -> textMode.setTextColor(Color.parseColor("#94A3B8"))            // Slate
                }

                // 2. Status Card: Navigation State
                if (navState.isReverseNavActive) {
                    textNavState.text = navState.navStateName
                    when (navState.navStateCode) {
                        0 -> textNavState.setTextColor(Color.parseColor("#10B981")) // ON_TRAIL: Emerald
                        1 -> textNavState.setTextColor(Color.parseColor("#F59E0B")) // DEVIATING: Amber
                        2 -> textNavState.setTextColor(Color.parseColor("#EF4444")) // OFF_TRAIL: Red
                        3 -> textNavState.setTextColor(Color.parseColor("#38BDF8")) // RECOVERING: Cyan
                        4 -> textNavState.setTextColor(Color.parseColor("#A855F7")) // RETURN_COMPLETE: Purple
                        else -> textNavState.setTextColor(Color.parseColor("#94A3B8"))
                    }
                } else {
                    textNavState.text = "--"
                    textNavState.setTextColor(Color.parseColor("#94A3B8"))
                }

                // 3. Status Card: Confidence
                val cMag = navState.cMag
                val confPct = (cMag * 100).coerceIn(0f, 100f).toInt()
                val confLevel = when {
                    confPct >= 60 -> "High"
                    confPct >= 30 -> "Medium"
                    else -> "Low"
                }
                textConfidence.text = "$confLevel ($confPct%)"
                when (confLevel) {
                    "High" -> textConfidence.setTextColor(Color.parseColor("#10B981"))
                    "Medium" -> textConfidence.setTextColor(Color.parseColor("#F59E0B"))
                    else -> textConfidence.setTextColor(Color.parseColor("#EF4444"))
                }

                // 4. Live Metrics
                textMetricSteps.text = "${navState.stepCount}"
                textMetricDistance.text = "%.1f m".format(navState.cumulativeDistance)
                textMetricPhoneHeading.text = "%.1f°".format(navState.headingDeg)

                if (navState.stepCount > 0 || !navState.isStationary) {
                    textMetricTravelHeading.text = "%.1f°".format(navState.travelHeadingDeg)
                } else {
                    textMetricTravelHeading.text = "--"
                }

                if (navState.isReverseNavActive) {
                    val sign = if (navState.crossTrackError >= 0) "+" else ""
                    textMetricXte.text = "$sign%.2f m".format(navState.crossTrackError)
                } else {
                    textMetricXte.text = "--"
                }

                // 5. Main Visual: Trail Canvas View
                val ptsCopied = NavigationNative.getTrailPoints(trailBuffer, 500)
                val trailStateStr = when {
                    navState.isReverseNavActive -> "RETURN_ACTIVE"
                    navState.isTrailRecording -> "RECORDING"
                    navState.isReturnReady -> "RETURN_READY"
                    else -> "IDLE"
                }
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

                // 6. Controls Enablement
                btnStartRecord.isEnabled = !navState.isTrailRecording && !navState.isReverseNavActive
                btnStopRecord.isEnabled = navState.isTrailRecording
                btnReturn.isEnabled = (navState.isReturnReady || navState.isReverseNavActive)
                btnReturn.text = if (navState.isReverseNavActive) "STOP RETURN" else "RETURN"
                btnReset.isEnabled = !navState.isTrailRecording

                // Diagnostic 1Hz logcat
                if (System.currentTimeMillis() - lastLogTimeMs >= 1000L) {
                    lastLogTimeMs = System.currentTimeMillis()
                    android.util.Log.i(
                        "BreadcrumbUI",
                        "Mode: $modeStr | Steps: ${navState.stepCount} | Dist: ${"%.2f".format(navState.cumulativeDistance)}m | TravelH: ${"%.1f".format(navState.travelHeadingDeg)}° | PhoneH: ${"%.1f".format(navState.headingDeg)}° | XTE: ${if (navState.isReverseNavActive) "%.2f".format(navState.crossTrackError) else "--"}m | Nav: ${navState.navStateName}"
                    )
                }
            }

            uiHandler.postDelayed(this, 250) // 4 Hz refresh
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
