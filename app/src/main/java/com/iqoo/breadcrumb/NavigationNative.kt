package com.iqoo.breadcrumb

/**
 * JNI adapter bridging the Android runtime to the C++ deterministic navigation engine.
 *
 * Implements Section 21.4 (JNI boundary) and Section 16 (Ordered trail memory) of gpt_technical_final.md.
 */
object NavigationNative {
    init {
        System.loadLibrary("breadcrumb_native")
    }

    /**
     * Sanity check version query.
     */
    external fun nativeGetVersion(): Int

    /**
     * Pushes an accelerometer sample into the native engine (triggers PDR step evaluation and trail appends).
     */
    external fun pushAccel(x: Float, y: Float, z: Float, tNs: Long)

    /**
     * Pushes a gyroscope sample into the native engine, triggering an AHRS update step.
     */
    external fun pushGyro(x: Float, y: Float, z: Float, tNs: Long)

    /**
     * Pushes a magnetometer sample into the native engine.
     */
    external fun pushMag(x: Float, y: Float, z: Float, tNs: Long)

    /**
     * Retrieves the current native engine state into a preallocated FloatArray (minimum 20 elements).
     *
     * State buffer index documentation (20 floats):
     * [0]  E (meters East)
     * [1]  N (meters North)
     * [2]  U (0.0f)
     * [3]  heading (radians, clockwise from North)
     * [4]  qw (attitude quaternion w)
     * [5]  qx (attitude quaternion x)
     * [6]  qy (attitude quaternion y)
     * [7]  qz (attitude quaternion z)
     * [8]  step_count (total detected steps)
     * [9]  stride_length (meters, last accepted step)
     * [10] cumulative_distance (meters)
     * [11] step_frequency (Hz)
     * [12] is_stationary (1.0f if stationary, 0.0f if moving/walking)
     * [13] Cmag (continuous magnetic reliability in [0, 1])
     * [14] gyro_bias_norm (rad/s)
     * [15] calibrated_B0 (µT, or -1.0f if uncalibrated)
     * [16] trail_point_count (number of breadcrumbs stored)
     * [17] trail_recording_state (0 = IDLE, 1 = RECORDING, 2 = RETURN_READY)
     * [18] journey_duration_sec (elapsed seconds since journey start)
     * [19] total_trail_distance (meters walked on recorded journey)
     *
     * @return Number of elements populated (20) or negative on error.
     */
    external fun nativeGetState(outBuffer: FloatArray): Int

    /**
     * Resets the native AHRS, PDR, trail, and stationary detector state.
     */
    external fun nativeReset()

    /**
     * Starts a new ordered breadcrumb trail recording. Resets PDR and initial point P0.
     */
    external fun startTrailRecording(): Boolean

    /**
     * Stops trail recording and freezes the breadcrumb trail in RETURN_READY state.
     */
    external fun stopTrailRecording(): Boolean

    /**
     * Resets the breadcrumb trail and PDR position back to IDLE.
     */
    external fun resetTrail()

    /**
     * Retrieves up to maxPoints breadcrumb points from native memory into preallocated buffer.
     * Each point occupies 4 consecutive floats: [east, north, heading, cumulativeDistance].
     *
     * @return Number of points copied.
     */
    external fun getTrailPoints(outFloats: FloatArray, maxPoints: Int): Int

    /**
     * Starts deterministic reverse breadcrumb navigation on frozen trail.
     */
    external fun startReverseNavigation(): Boolean

    /**
     * Stops reverse breadcrumb navigation.
     */
    external fun stopReverseNavigation()

    /**
     * Checks if reverse breadcrumb navigation is currently active.
     */
    external fun isReverseNavigationActive(): Boolean

    /**
     * Sets Madgwick AHRS variant (0 = Variant A, 1 = Variant B).
     */
    external fun setMadgwickVariant(variant: Int)

    /**
     * Gets current Madgwick AHRS variant (0 = Variant A, 1 = Variant B).
     */
    external fun getMadgwickVariant(): Int
}


/**
 * High-level immutable representation of the native navigation engine state.
 */
data class NativeNavState(
    val east: Float,
    val north: Float,
    val up: Float,
    val headingRad: Float,
    val headingDeg: Float,
    val qw: Float,
    val qx: Float,
    val qy: Float,
    val qz: Float,
    val stepCount: Int,
    val strideLength: Float,
    val cumulativeDistance: Float,
    val stepFrequency: Float,
    val isStationary: Boolean,
    val cMag: Float,
    val gyroBiasNorm: Float,
    val calibratedB0: Float,
    val trailPointCount: Int,
    val trailStateCode: Int,
    val journeyDurationSec: Float,
    val totalTrailDistance: Float,
    val isReverseNavActive: Boolean = false,
    val reverseProgressIndex: Int = 0,
    val matchedSegmentIndex: Int = 0,
    val projEast: Float = 0.0f,
    val projNorth: Float = 0.0f,
    val carrotEast: Float = 0.0f,
    val carrotNorth: Float = 0.0f,
    val crossTrackError: Float = 0.0f, // signed meters (+ = trail to user's left)
    val desiredHeadingRad: Float = 0.0f,
    val steeringErrorRad: Float = 0.0f,
    val navStateCode: Int = 0, // 0=ON_TRAIL, 1=DEVIATING, 2=OFF_TRAIL, 3=RECOVERING, 4=RETURN_COMPLETE
    val isReturnComplete: Boolean = false,
    val guidanceModeCode: Int = 0, // 0=NORMAL_CARROT, 1=LATERAL_BLEND, 2=DIRECT_RECOVERY, 3=ARRIVED
    val turnCommandCode: Int = 0,  // 0=STRAIGHT, 1=TURN_LEFT, 2=TURN_RIGHT, 3=U_TURN
    val recoveryActive: Boolean = false,
    val recoveryTargetEast: Float = 0.0f,
    val recoveryTargetNorth: Float = 0.0f,
    val guidanceVectorEast: Float = 0.0f,
    val guidanceVectorNorth: Float = 0.0f
) {
    val isTrailRecording: Boolean get() = trailStateCode == 1
    val isReturnReady: Boolean get() = trailStateCode == 2
    val isTrailIdle: Boolean get() = trailStateCode == 0

    val navStateName: String get() = when (navStateCode) {
        0 -> "ON_TRAIL"
        1 -> "DEVIATING"
        2 -> "OFF_TRAIL"
        3 -> "RECOVERING"
        4 -> "RETURN_COMPLETE"
        else -> "UNKNOWN"
    }

    val guidanceModeName: String get() = when (guidanceModeCode) {
        0 -> "NORMAL_CARROT"
        1 -> "LATERAL_BLEND"
        2 -> "DIRECT_RECOVERY"
        3 -> "ARRIVED"
        else -> "UNKNOWN"
    }

    val turnCommandName: String get() = when (turnCommandCode) {
        0 -> "STRAIGHT"
        1 -> "TURN_LEFT"
        2 -> "TURN_RIGHT"
        3 -> "U_TURN"
        else -> "UNKNOWN"
    }

    val desiredHeadingDeg: Float get() = Math.toDegrees(desiredHeadingRad.toDouble()).toFloat()
    val steeringErrorDeg: Float get() = Math.toDegrees(steeringErrorRad.toDouble()).toFloat()

    companion object {
        fun fromBuffer(buffer: FloatArray): NativeNavState {
            val headingRad = buffer[3]
            val headingDeg = Math.toDegrees(headingRad.toDouble()).toFloat()
            val pointCount = if (buffer.size >= 20) buffer[16].toInt() else 0
            val stateCode = if (buffer.size >= 20) buffer[17].toInt() else 0
            val duration = if (buffer.size >= 20) buffer[18] else 0.0f
            val trailDist = if (buffer.size >= 20) buffer[19] else 0.0f

            val isRevActive = if (buffer.size >= 32) buffer[20] > 0.5f else false
            val revProgIdx = if (buffer.size >= 32) buffer[21].toInt() else 0
            val matchedSegIdx = if (buffer.size >= 32) buffer[22].toInt() else 0
            val projE = if (buffer.size >= 32) buffer[23] else 0.0f
            val projN = if (buffer.size >= 32) buffer[24] else 0.0f
            val carrotE = if (buffer.size >= 32) buffer[25] else 0.0f
            val carrotN = if (buffer.size >= 32) buffer[26] else 0.0f
            val xte = if (buffer.size >= 32) buffer[27] else 0.0f
            val desHeadingRad = if (buffer.size >= 32) buffer[28] else 0.0f
            val steerErrRad = if (buffer.size >= 32) buffer[29] else 0.0f
            val navCode = if (buffer.size >= 32) buffer[30].toInt() else 0
            val retComp = if (buffer.size >= 32) buffer[31] > 0.5f else false

            val gMode = if (buffer.size >= 39) buffer[32].toInt() else 0
            val turnCmd = if (buffer.size >= 39) buffer[33].toInt() else 0
            val recovAct = if (buffer.size >= 39) buffer[34] > 0.5f else false
            val recovTE = if (buffer.size >= 39) buffer[35] else 0.0f
            val recovTN = if (buffer.size >= 39) buffer[36] else 0.0f
            val gVecE = if (buffer.size >= 39) buffer[37] else 0.0f
            val gVecN = if (buffer.size >= 39) buffer[38] else 0.0f

            return NativeNavState(
                east = buffer[0],
                north = buffer[1],
                up = buffer[2],
                headingRad = headingRad,
                headingDeg = headingDeg,
                qw = buffer[4],
                qx = buffer[5],
                qy = buffer[6],
                qz = buffer[7],
                stepCount = buffer[8].toInt(),
                strideLength = buffer[9],
                cumulativeDistance = buffer[10],
                stepFrequency = buffer[11],
                isStationary = buffer[12] > 0.5f,
                cMag = buffer[13],
                gyroBiasNorm = buffer[14],
                calibratedB0 = buffer[15],
                trailPointCount = pointCount,
                trailStateCode = stateCode,
                journeyDurationSec = duration,
                totalTrailDistance = trailDist,
                isReverseNavActive = isRevActive,
                reverseProgressIndex = revProgIdx,
                matchedSegmentIndex = matchedSegIdx,
                projEast = projE,
                projNorth = projN,
                carrotEast = carrotE,
                carrotNorth = carrotN,
                crossTrackError = xte,
                desiredHeadingRad = desHeadingRad,
                steeringErrorRad = steerErrRad,
                navStateCode = navCode,
                isReturnComplete = retComp,
                guidanceModeCode = gMode,
                turnCommandCode = turnCmd,
                recoveryActive = recovAct,
                recoveryTargetEast = recovTE,
                recoveryTargetNorth = recovTN,
                guidanceVectorEast = gVecE,
                guidanceVectorNorth = gVecN
            )
        }
    }
}

/**
 * Representation of an individual breadcrumb point in the ordered trail.
 */
data class BreadcrumbPoint(
    val sequenceIndex: Int,
    val east: Float,
    val north: Float,
    val heading: Float,
    val distance: Float
)
