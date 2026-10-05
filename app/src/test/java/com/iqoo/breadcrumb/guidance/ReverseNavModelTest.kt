package com.iqoo.breadcrumb.guidance

import com.iqoo.breadcrumb.NativeNavState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ReverseNavModelTest {

    @Test
    fun testNativeNavStateParsingFrom32FloatBuffer() {
        val buffer = FloatArray(32)
        // 0..19: PDR & Trail fields
        buffer[0] = 12.0f      // East
        buffer[1] = 24.0f      // North
        buffer[2] = 0.0f       // Up
        buffer[3] = 0.785398f  // Heading 45 deg
        buffer[4] = 1.0f       // qw
        buffer[5] = 0.0f       // qx
        buffer[6] = 0.0f       // qy
        buffer[7] = 0.0f       // qz
        buffer[8] = 30.0f      // stepCount
        buffer[9] = 0.75f      // strideLength
        buffer[10] = 22.5f     // cumulativeDistance
        buffer[11] = 1.8f      // stepFrequency
        buffer[12] = 0.0f      // isStationary
        buffer[13] = 0.92f     // cMag
        buffer[14] = 0.001f    // gyroBiasNorm
        buffer[15] = 45.0f     // calibratedB0
        buffer[16] = 31.0f     // trailPointCount
        buffer[17] = 2.0f      // trailStateCode (RETURN_READY)
        buffer[18] = 20.0f     // journeyDurationSec
        buffer[19] = 22.5f     // totalTrailDistance

        // 20..31: M5 Reverse Navigation fields
        buffer[20] = 1.0f      // isReverseNavActive
        buffer[21] = 5.0f      // reverseProgressIndex
        buffer[22] = 5.0f      // matchedSegmentIndex
        buffer[23] = 11.5f     // projEast
        buffer[24] = 24.0f     // projNorth
        buffer[25] = 8.0f      // carrotEast
        buffer[26] = 20.0f     // carrotNorth
        buffer[27] = 0.5f      // crossTrackError (+0.5m -> trail is to user's left)
        buffer[28] = -2.35619f // desiredHeading (-135 deg)
        buffer[29] = -3.14159f // steeringError (-180 deg)
        buffer[30] = 0.0f      // navStateCode (ON_TRAIL)
        buffer[31] = 0.0f      // isReturnComplete (false)

        val state = NativeNavState.fromBuffer(buffer)

        assertTrue(state.isReturnReady)
        assertTrue(state.isReverseNavActive)
        assertEquals(5, state.reverseProgressIndex)
        assertEquals(5, state.matchedSegmentIndex)
        assertEquals(11.5f, state.projEast, 1e-4f)
        assertEquals(24.0f, state.projNorth, 1e-4f)
        assertEquals(8.0f, state.carrotEast, 1e-4f)
        assertEquals(20.0f, state.carrotNorth, 1e-4f)
        assertEquals(0.5f, state.crossTrackError, 1e-4f)
        assertEquals(-135.0f, state.desiredHeadingDeg, 0.1f)
        assertEquals(-180.0f, state.steeringErrorDeg, 0.1f)
        assertEquals(0, state.navStateCode)
        assertEquals("ON_TRAIL", state.navStateName)
        assertFalse(state.isReturnComplete)
    }

    @Test
    fun testNavigationStateCodesAndNames() {
        val states = mapOf(
            0 to "ON_TRAIL",
            1 to "DEVIATING",
            2 to "OFF_TRAIL",
            3 to "RECOVERING",
            4 to "RETURN_COMPLETE"
        )

        val buf = FloatArray(32)
        buf[20] = 1.0f // active

        for ((code, name) in states) {
            buf[30] = code.toFloat()
            buf[31] = if (code == 4) 1.0f else 0.0f
            val state = NativeNavState.fromBuffer(buf)
            assertEquals(name, state.navStateName)
            if (code == 4) {
                assertTrue(state.isReturnComplete)
            } else {
                assertFalse(state.isReturnComplete)
            }
        }
    }

    @Test
    fun testBackwardCompatibilityWith20FloatBuffer() {
        val buffer = FloatArray(20)
        buffer[0] = 5.0f
        buffer[1] = 10.0f
        buffer[17] = 2.0f // RETURN_READY

        val state = NativeNavState.fromBuffer(buffer)
        assertEquals(5.0f, state.east, 1e-4f)
        assertEquals(10.0f, state.north, 1e-4f)
        assertTrue(state.isReturnReady)
        assertFalse(state.isReverseNavActive)
        assertEquals(0, state.navStateCode)
        assertEquals("ON_TRAIL", state.navStateName)
        assertFalse(state.isReturnComplete)
    }
}
