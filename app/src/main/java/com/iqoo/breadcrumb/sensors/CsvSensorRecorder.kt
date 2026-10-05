package com.iqoo.breadcrumb.sensors

import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * High-performance, non-blocking raw sensor CSV recorder.
 *
 * Enqueues sensor samples from the 50 Hz callbacks into a bounded ring buffer
 * and drains them to disk on a dedicated I/O thread.
 *
 * Adheres strictly to the replay-compatible schema defined in gpt_technical_final.md:
 * Header: timestamp_ns,sensor_type,x,y,z
 */
class CsvSensorRecorder {

    private val isRecording = AtomicBoolean(false)
    private val sampleCount = AtomicLong(0L)

    private var currentFile: File? = null
    private var writer: BufferedWriter? = null
    private var writerExecutor: ExecutorService? = null

    // Bounded queue to prevent unbounded memory growth (capacity: 4096 samples ≈ 30-40 seconds of buffering)
    private val eventQueue = ArrayBlockingQueue<RawSensorEvent>(4096)

    private var startWallTimeMs: Long = 0L
    private var firstEventTimestampNs: Long = 0L
    private var lastEventTimestampNs: Long = 0L

    data class RecordingSummary(
        val file: File,
        val sampleCount: Long,
        val durationMs: Long,
        val fileSizeBytes: Long
    )

    /**
     * Starts recording to a timestamped CSV file in the specified directory.
     */
    @Synchronized
    fun startRecording(outputDir: File, prefix: String = "walk_recording"): File {
        if (isRecording.get()) {
            return currentFile ?: throw IllegalStateException("Recorder already running")
        }

        if (!outputDir.exists()) {
            outputDir.mkdirs()
        }

        val timestampStr = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val file = File(outputDir, "${prefix}_$timestampStr.csv")

        val fw = FileWriter(file, false)
        val bw = BufferedWriter(fw, 16384) // 16 KB write buffer

        // Write authoritative CSV header
        bw.write("timestamp_ns,sensor_type,x,y,z\n")
        bw.flush()

        currentFile = file
        writer = bw
        sampleCount.set(0L)
        firstEventTimestampNs = 0L
        lastEventTimestampNs = 0L
        startWallTimeMs = System.currentTimeMillis()
        eventQueue.clear()

        val executor = Executors.newSingleThreadExecutor { r ->
            Thread(r, "SensorCsvWriter").apply { priority = Thread.MIN_PRIORITY }
        }
        writerExecutor = executor

        isRecording.set(true)

        // Launch draining worker
        executor.submit {
            drainQueueToFile(bw)
        }

        return file
    }

    /**
     * Non-blocking record call invoked from sensor callbacks.
     */
    fun recordEvent(event: RawSensorEvent) {
        if (!isRecording.get()) return

        if (firstEventTimestampNs == 0L) {
            firstEventTimestampNs = event.timestampNs
        }
        lastEventTimestampNs = event.timestampNs

        // Offer without blocking sensor thread; drop if queue saturated
        val accepted = eventQueue.offer(event)
        if (accepted) {
            sampleCount.incrementAndGet()
        }
    }

    /**
     * Stops recording, flushes all remaining samples, and closes the writer.
     */
    @Synchronized
    fun stopRecording(): RecordingSummary? {
        if (!isRecording.compareAndSet(true, false)) {
            return null
        }

        val file = currentFile ?: return null

        try {
            // Shutdown writer executor and wait for queue to drain
            writerExecutor?.shutdown()
            writerExecutor?.awaitTermination(2, TimeUnit.SECONDS)
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        } finally {
            writerExecutor = null
        }

        // Final flush and close
        try {
            writer?.flush()
            writer?.close()
        } catch (_: IOException) {
        } finally {
            writer = null
        }

        val duration = System.currentTimeMillis() - startWallTimeMs
        val finalCount = sampleCount.get()
        val sizeBytes = file.length()

        return RecordingSummary(
            file = file,
            sampleCount = finalCount,
            durationMs = duration,
            fileSizeBytes = sizeBytes
        )
    }

    fun isRecordingActive(): Boolean = isRecording.get()

    fun getRecordedSampleCount(): Long = sampleCount.get()

    fun getCurrentFile(): File? = currentFile

    private fun drainQueueToFile(bw: BufferedWriter) {
        var unwrittenCount = 0
        try {
            while (isRecording.get() || eventQueue.isNotEmpty()) {
                val event = eventQueue.poll(100, TimeUnit.MILLISECONDS) ?: continue
                bw.write(event.toCsvLine())
                bw.newLine()
                unwrittenCount++

                if (unwrittenCount >= 100) {
                    bw.flush()
                    unwrittenCount = 0
                }
            }
            bw.flush()
        } catch (_: IOException) {
            // I/O interruption during recording
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }
}
