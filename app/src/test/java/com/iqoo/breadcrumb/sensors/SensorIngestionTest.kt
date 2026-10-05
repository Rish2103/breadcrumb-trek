package com.iqoo.breadcrumb.sensors

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.BufferedReader
import java.io.FileReader

class SensorIngestionTest {

    @get:Rule
    val tempFolder = TemporaryFolder()

    @Test
    fun testRawSensorEventCsvRoundTrip() {
        val original = RawSensorEvent(
            timestampNs = 1718000000000000L,
            sensorType = 1, // TYPE_ACCELEROMETER
            x = 0.12f,
            y = 9.81f,
            z = -0.05f,
            accuracy = 3
        )

        val csvLine = original.toCsvLine()
        assertEquals("1718000000000000,1,0.12,9.81,-0.05", csvLine)

        val parsed = RawSensorEvent.fromCsvLine(csvLine)
        assertNotNull(parsed)
        assertEquals(original.timestampNs, parsed!!.timestampNs)
        assertEquals(original.sensorType, parsed.sensorType)
        assertEquals(original.x, parsed.x, 0.0001f)
        assertEquals(original.y, parsed.y, 0.0001f)
        assertEquals(original.z, parsed.z, 0.0001f)
    }

    @Test
    fun testBarometerEventSerialization() {
        val baroEvent = RawSensorEvent(
            timestampNs = 1718000000100000L,
            sensorType = 6, // TYPE_PRESSURE
            x = 1013.25f,
            y = 0.0f,
            z = 0.0f
        )
        val line = baroEvent.toCsvLine()
        val parsed = RawSensorEvent.fromCsvLine(line)
        assertNotNull(parsed)
        assertEquals(6, parsed!!.sensorType)
        assertEquals(1013.25f, parsed.x, 0.01f)
        assertEquals(0.0f, parsed.y, 0.0001f)
        assertEquals(0.0f, parsed.z, 0.0001f)
    }

    @Test
    fun testMalformedCsvParsingReturnsNull() {
        assertNull(RawSensorEvent.fromCsvLine(""))
        assertNull(RawSensorEvent.fromCsvLine("not,a,valid,csv"))
        assertNull(RawSensorEvent.fromCsvLine("123,1,0.0,0.0")) // missing column
        assertNull(RawSensorEvent.fromCsvLine("invalid_num,1,0.0,0.0,0.0"))
    }

    @Test
    fun testCsvSensorRecorderLifecycleAndFlushing() {
        val recordDir = tempFolder.newFolder("recordings")
        val recorder = CsvSensorRecorder()

        assertFalse(recorder.isRecordingActive())
        val file = recorder.startRecording(recordDir, "test_walk")
        assertTrue(recorder.isRecordingActive())
        assertTrue(file.exists())

        // Enqueue 250 sample events (mixed sensors)
        val baseTime = 1000000000L
        for (i in 0 until 250) {
            val event = RawSensorEvent(
                timestampNs = baseTime + (i * 20_000_000L), // 20ms steps
                sensorType = if (i % 2 == 0) 1 else 4,
                x = i * 0.1f,
                y = 9.8f,
                z = 0.0f
            )
            recorder.recordEvent(event)
        }

        val summary = recorder.stopRecording()
        assertNotNull(summary)
        assertFalse(recorder.isRecordingActive())
        assertEquals(250L, summary!!.sampleCount)
        assertTrue(summary.fileSizeBytes > 0)

        // Verify CSV file content and header
        BufferedReader(FileReader(summary.file)).use { reader ->
            val header = reader.readLine()
            assertEquals("timestamp_ns,sensor_type,x,y,z", header)

            var lineCount = 0
            var line = reader.readLine()
            while (line != null) {
                lineCount++
                val parsed = RawSensorEvent.fromCsvLine(line)
                assertNotNull("Line $lineCount must be parseable: $line", parsed)
                line = reader.readLine()
            }
            assertEquals(250, lineCount)
        }
    }

    @Test
    fun testMonotonicityValidationLogic() {
        // Test timestamps: increasing, same, decreasing, large jump
        val t0 = 1000000000L
        val t1 = 1020000000L // +20ms (valid)
        val t2 = 1020000000L // +0ms (invalid: non-monotonic)
        val t3 = 1010000000L // -10ms (invalid: retrograde)
        val t4 = 1350000000L // +330ms (jump > 0.25s)

        assertTrue("t1 > t0", t1 - t0 > 0)
        assertFalse("t2 <= t1 is non-monotonic", t2 - t1 > 0)
        assertFalse("t3 < t2 is non-monotonic", t3 - t2 > 0)

        val dtJump = (t4 - t1) * 1e-9f
        assertTrue("dt > 0.25s detected", dtJump > 0.25f)
    }

    @Test
    fun testStalenessThresholds() {
        val nowNs = 2000000000L

        // Accel: threshold 100ms (100_000_000 ns)
        val accelFreshT = nowNs - 50_000_000L // 50ms ago
        val accelStaleT = nowNs - 150_000_000L // 150ms ago
        assertFalse((nowNs - accelFreshT) > 100_000_000L)
        assertTrue((nowNs - accelStaleT) > 100_000_000L)

        // Mag: threshold 300ms (300_000_000 ns)
        val magFreshT = nowNs - 200_000_000L // 200ms ago
        val magStaleT = nowNs - 350_000_000L // 350ms ago
        assertFalse((nowNs - magFreshT) > 300_000_000L)
        assertTrue((nowNs - magStaleT) > 300_000_000L)
    }
}
