package com.iqoo.breadcrumb.trail

import com.iqoo.breadcrumb.BreadcrumbPoint
import com.iqoo.breadcrumb.NativeNavState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class TrailModelTest {

    @Test
    fun testNativeNavStateParsingFrom20FloatBuffer() {
        val buffer = FloatArray(20)
        buffer[0] = 5.25f       // East
        buffer[1] = 12.50f      // North
        buffer[2] = 0.0f        // Up
        buffer[3] = 1.5707963f  // Heading (~90 deg)
        buffer[4] = 1.0f        // qw
        buffer[5] = 0.0f        // qx
        buffer[6] = 0.0f        // qy
        buffer[7] = 0.0f        // qz
        buffer[8] = 42.0f       // step_count
        buffer[9] = 0.72f       // stride_length
        buffer[10] = 30.24f     // cumulative_distance
        buffer[11] = 1.8f       // step_frequency
        buffer[12] = 0.0f       // is_stationary (moving)
        buffer[13] = 0.95f      // cMag
        buffer[14] = 0.002f     // gyro_bias_norm
        buffer[15] = 46.5f      // calibrated_B0
        buffer[16] = 43.0f      // trail_point_count (P0 + 42 steps)
        buffer[17] = 1.0f       // trail_recording_state (RECORDING)
        buffer[18] = 23.4f      // journey_duration_sec
        buffer[19] = 30.24f     // total_trail_distance

        val state = NativeNavState.fromBuffer(buffer)

        assertEquals(5.25f, state.east, 1e-4f)
        assertEquals(12.50f, state.north, 1e-4f)
        assertEquals(90.0f, state.headingDeg, 0.1f)
        assertEquals(42, state.stepCount)
        assertEquals(0.72f, state.strideLength, 1e-4f)
        assertEquals(30.24f, state.cumulativeDistance, 1e-4f)
        assertFalse(state.isStationary)
        assertEquals(43, state.trailPointCount)
        assertEquals(1, state.trailStateCode)
        assertTrue(state.isTrailRecording)
        assertFalse(state.isReturnReady)
        assertFalse(state.isTrailIdle)
        assertEquals(23.4f, state.journeyDurationSec, 1e-4f)
        assertEquals(30.24f, state.totalTrailDistance, 1e-4f)
    }

    @Test
    fun testTrailRecordingStateTransitions() {
        val idleBuffer = FloatArray(20).apply { this[17] = 0.0f }
        val idleState = NativeNavState.fromBuffer(idleBuffer)
        assertTrue(idleState.isTrailIdle)
        assertFalse(idleState.isTrailRecording)
        assertFalse(idleState.isReturnReady)

        val recordingBuffer = FloatArray(20).apply { this[17] = 1.0f }
        val recState = NativeNavState.fromBuffer(recordingBuffer)
        assertFalse(recState.isTrailIdle)
        assertTrue(recState.isTrailRecording)
        assertFalse(recState.isReturnReady)

        val returnBuffer = FloatArray(20).apply { this[17] = 2.0f }
        val retState = NativeNavState.fromBuffer(returnBuffer)
        assertFalse(retState.isTrailIdle)
        assertFalse(retState.isTrailRecording)
        assertTrue(retState.isReturnReady)
    }

    @Test
    fun testBreadcrumbPointOrderingAndPreservation() {
        val points = mutableListOf<BreadcrumbPoint>()
        for (i in 0 until 10) {
            points.add(
                BreadcrumbPoint(
                    sequenceIndex = i,
                    east = i * 0.7f,
                    north = i * 0.5f,
                    heading = 0.8f,
                    distance = i * 0.86f
                )
            )
        }

        assertEquals(10, points.size)
        for (i in 0 until 10) {
            assertEquals(i, points[i].sequenceIndex)
        }

        // Return order verification (reverse navigation requirement)
        val returnTraversal = points.asReversed()
        assertEquals(9, returnTraversal.first().sequenceIndex)
        assertEquals(0, returnTraversal.last().sequenceIndex)
    }
}
