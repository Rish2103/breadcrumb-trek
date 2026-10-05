package com.iqoo.breadcrumb.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Lightweight relative 2D breadcrumb trail visualizer per gpt_technical_final.md §16.
 *
 * Visualizes local East/North trajectory relative to start (0, 0):
 * - Monotonic breadcrumb path (P0 → P1 → ... → Pn)
 * - Start anchor point (green)
 * - Latest estimated position with heading indicator (amber)
 * - 1:1 aspect ratio grid scaling without external map dependencies
 */
class TrailCanvasView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private val bgPaint = Paint().apply {
        color = Color.parseColor("#0F172A")
        style = Paint.Style.FILL
    }

    private val gridPaint = Paint().apply {
        color = Color.parseColor("#1E293B")
        style = Paint.Style.STROKE
        strokeWidth = 2f
    }

    private val trailPathPaint = Paint().apply {
        color = Color.parseColor("#38BDF8")
        style = Paint.Style.STROKE
        strokeWidth = 6f
        isAntiAlias = true
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }

    private val pointPaint = Paint().apply {
        color = Color.parseColor("#0284C7")
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    private val startPaint = Paint().apply {
        color = Color.parseColor("#10B981") // Emerald
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    private val currentPosPaint = Paint().apply {
        color = Color.parseColor("#F59E0B") // Amber
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    private val headingLinePaint = Paint().apply {
        color = Color.parseColor("#FBBF24")
        style = Paint.Style.STROKE
        strokeWidth = 4f
        isAntiAlias = true
    }

    private val textPaint = Paint().apply {
        color = Color.parseColor("#94A3B8")
        textSize = 28f
        isAntiAlias = true
    }

    private val carrotPaint = Paint().apply {
        color = Color.parseColor("#EC4899") // Pink/Magenta for Look-Ahead Carrot
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    private val projPaint = Paint().apply {
        color = Color.parseColor("#06B6D4") // Cyan
        style = Paint.Style.STROKE
        strokeWidth = 3f
        isAntiAlias = true
    }

    private val guidanceLinePaint = Paint().apply {
        color = Color.parseColor("#F472B6")
        style = Paint.Style.STROKE
        strokeWidth = 3f
        isAntiAlias = true
    }

    // Preallocated coordinate buffer (up to 500 breadcrumbs * 4 floats)
    private val pointsBuffer = FloatArray(2000)
    private var pointCount = 0

    private var currentEast = 0.0f
    private var currentNorth = 0.0f
    private var currentHeadingRad = 0.0f
    private var recordingStateStr = "IDLE"

    // Reverse Navigation Telemetry
    private var isReverseNavActive = false
    private var carrotEast = 0.0f
    private var carrotNorth = 0.0f
    private var projEast = 0.0f
    private var projNorth = 0.0f
    private var crossTrackError = 0.0f
    private var navStateStr = "ON_TRAIL"
    private var guidanceModeStr = "NORMAL_CARROT"
    private var turnCommandStr = "STRAIGHT"
    private var recoveryActive = false
    private var guidanceVecEast = 0.0f
    private var guidanceVecNorth = 0.0f

    private val path = Path()

    /**
     * Updates visualizer telemetry without heap allocation.
     */
    fun updateTrail(
        rawPoints: FloatArray,
        count: Int,
        curE: Float,
        curN: Float,
        curHeading: Float,
        stateStr: String,
        isRevActive: Boolean = false,
        cE: Float = 0.0f,
        cN: Float = 0.0f,
        pE: Float = 0.0f,
        pN: Float = 0.0f,
        xte: Float = 0.0f,
        nState: String = "ON_TRAIL",
        gMode: String = "NORMAL_CARROT",
        tCmd: String = "STRAIGHT",
        recovAct: Boolean = false,
        gE: Float = 0.0f,
        gN: Float = 0.0f
    ) {
        val ptsToCopy = min(count, pointsBuffer.size / 4)
        System.arraycopy(rawPoints, 0, pointsBuffer, 0, ptsToCopy * 4)
        pointCount = ptsToCopy
        currentEast = curE
        currentNorth = curN
        currentHeadingRad = curHeading
        recordingStateStr = stateStr
        isReverseNavActive = isRevActive
        carrotEast = cE
        carrotNorth = cN
        projEast = pE
        projNorth = pN
        crossTrackError = xte
        navStateStr = nState
        guidanceModeStr = gMode
        turnCommandStr = tCmd
        recoveryActive = recovAct
        guidanceVecEast = gE
        guidanceVecNorth = gN
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        if (w <= 0 || h <= 0) return

        // 1. Fill background
        canvas.drawRect(0f, 0f, w, h, bgPaint)

        // 2. Compute bounding box in meters (including start at (0,0) and current pos)
        var minE = min(0.0f, currentEast)
        var maxE = max(0.0f, currentEast)
        var minN = min(0.0f, currentNorth)
        var maxN = max(0.0f, currentNorth)

        for (i in 0 until pointCount) {
            val pe = pointsBuffer[i * 4 + 0]
            val pn = pointsBuffer[i * 4 + 1]
            minE = min(minE, pe)
            maxE = max(maxE, pe)
            minN = min(minN, pn)
            maxN = max(maxN, pn)
        }

        // Maintain minimum 10m x 10m viewport span with padding
        val spanE = max(maxE - minE, 10.0f)
        val spanN = max(maxN - minN, 10.0f)
        val maxSpan = max(spanE, spanN) * 1.3f // 30% margin

        val centerE = (minE + maxE) / 2f
        val centerN = (minN + maxN) / 2f

        val scale = min(w, h) / maxSpan
        val cx = w / 2f
        val cy = h / 2f

        fun toScreenX(e: Float): Float = cx + (e - centerE) * scale
        fun toScreenY(n: Float): Float = cy - (n - centerN) * scale // North is Up

        // 3. Draw grid lines
        val gridStepM = 5.0f
        var gm = -25.0f
        while (gm <= 25.0f) {
            val gx = toScreenX(gm)
            if (gx in 0f..w) canvas.drawLine(gx, 0f, gx, h, gridPaint)
            val gy = toScreenY(gm)
            if (gy in 0f..h) canvas.drawLine(0f, gy, w, gy, gridPaint)
            gm += gridStepM
        }

        // 4. Draw trail polyline
        path.reset()
        if (pointCount > 0) {
            val startX = toScreenX(pointsBuffer[0])
            val startY = toScreenY(pointsBuffer[1])
            path.moveTo(startX, startY)

            for (i in 1 until pointCount) {
                val px = toScreenX(pointsBuffer[i * 4 + 0])
                val py = toScreenY(pointsBuffer[i * 4 + 1])
                path.lineTo(px, py)
            }
            canvas.drawPath(path, trailPathPaint)

            // Draw small breadcrumb dots along the trail
            for (i in 0 until pointCount) {
                val px = toScreenX(pointsBuffer[i * 4 + 0])
                val py = toScreenY(pointsBuffer[i * 4 + 1])
                canvas.drawCircle(px, py, 6f, pointPaint)
            }
        }

        // 5. Draw Start anchor point P0 at (0,0)
        val p0x = toScreenX(0.0f)
        val p0y = toScreenY(0.0f)
        canvas.drawCircle(p0x, p0y, 14f, startPaint)
        canvas.drawText("START (0,0)", p0x + 18f, p0y + 10f, textPaint)

        // 6. Draw Current Position with Heading Indicator
        val curX = toScreenX(currentEast)
        val curY = toScreenY(currentNorth)
        canvas.drawCircle(curX, curY, 16f, currentPosPaint)

        // Heading arrow (bearing clockwise from North: dx = sin(ψ), dy = -cos(ψ))
        val arrowLen = 32f
        val hx = curX + arrowLen * sin(currentHeadingRad)
        val hy = curY - arrowLen * cos(currentHeadingRad)
        canvas.drawLine(curX, curY, hx, hy, headingLinePaint)

        // 7. Draw Reverse Navigation Overlays per M6 Guidance Mode (§18.1-18.4)
        if (isReverseNavActive) {
            val qx = toScreenX(projEast)
            val qy = toScreenY(projNorth)
            canvas.drawCircle(qx, qy, 8f, projPaint)

            when (guidanceModeStr) {
                "NORMAL_CARROT" -> {
                    val cx = toScreenX(carrotEast)
                    val cy = toScreenY(carrotNorth)
                    canvas.drawCircle(cx, cy, 14f, carrotPaint)
                    canvas.drawText("CARROT", cx + 18f, cy + 10f, textPaint)
                    // Guidance line to look-ahead carrot C
                    canvas.drawLine(curX, curY, cx, cy, guidanceLinePaint)
                }
                "LATERAL_BLEND" -> {
                    // Lateral guidance direction ray from user position
                    val rayLenPx = 40f
                    val gx = curX + rayLenPx * guidanceVecEast
                    val gy = curY - rayLenPx * guidanceVecNorth
                    canvas.drawLine(curX, curY, gx, gy, guidanceLinePaint)
                    canvas.drawCircle(gx, gy, 6f, carrotPaint)
                }
                "DIRECT_RECOVERY" -> {
                    // Direct recovery ray from user position to projection Q
                    canvas.drawLine(curX, curY, qx, qy, guidanceLinePaint)
                    canvas.drawText("RECOVERY Q", qx + 14f, qy + 10f, textPaint)
                }
                "ARRIVED" -> {
                    // No guidance ray after RETURN_COMPLETE
                }
            }
        }

        // 8. Overlay Legend / Status
        val statusText = if (isReverseNavActive) {
            "RETURN: $navStateStr [$guidanceModeStr] | CMD: $turnCommandStr | XTE: ${"%.2f".format(crossTrackError)}m"
        } else {
            "TRAIL: $recordingStateStr | Pts: $pointCount | Scale: ±${(maxSpan/2).toInt()}m"
        }
        canvas.drawText(statusText, 24f, 44f, textPaint)
    }
}
