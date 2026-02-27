package dji.sampleV5.aircraft.detection

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.graphics.Typeface
import android.util.AttributeSet
import android.view.View

/**
 * Transparent overlay view that draws bounding boxes and labels produced by
 * [RhinoYoloDetector] on top of the drone's live video feed.
 *
 * The view reads from [DetectionState] on the main thread's `onDraw` call—
 * no locking required because [DetectionState] uses an [AtomicReference].
 *
 * Stale-result protection: if the last inference update is older than
 * [DetectionState.MAX_RESULT_AGE_MS] the view draws nothing, so boxes
 * disappear automatically when the camera points away from wildlife.
 *
 * ## Layout usage
 * Place this view directly on top of the FPV widget in the ConstraintLayout:
 * ```xml
 * <dji.sampleV5.aircraft.detection.DetectionOverlayView
 *     android:id="@+id/view_detection_overlay"
 *     android:layout_width="0dp"
 *     android:layout_height="0dp"
 *     android:visibility="gone"
 *     app:layout_constraintTop_toTopOf="@id/fpv_holder"
 *     app:layout_constraintStart_toStartOf="@id/fpv_holder"
 *     app:layout_constraintEnd_toEndOf="@id/fpv_holder"
 *     app:layout_constraintBottom_toBottomOf="@id/fpv_holder" />
 * ```
 */
class DetectionOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0
) : View(context, attrs, defStyle) {

    // ── Paints ─────────────────────────────────────────────────────────────

    private val boxPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style       = Paint.Style.STROKE
        strokeWidth = 3f
        color       = Color.parseColor("#FF4CAF50")   // green default
    }

    private val labelBgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.argb(180, 0, 0, 0)              // semi-transparent black
    }

    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style    = Paint.Style.FILL
        color    = Color.WHITE
        textSize = 32f
        typeface = Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
    }

    // Class-index → colour cycling (vivid palette)
    private val classColours = intArrayOf(
        Color.parseColor("#4CAF50"),   // green
        Color.parseColor("#F44336"),   // red
        Color.parseColor("#2196F3"),   // blue
        Color.parseColor("#FF9800"),   // orange
        Color.parseColor("#9C27B0"),   // purple
        Color.parseColor("#00BCD4"),   // cyan
        Color.parseColor("#FFEB3B"),   // yellow
        Color.parseColor("#FF5722"),   // deep orange
    )

    // ── State ──────────────────────────────────────────────────────────────

    /** Snapshot written from any thread, read on main/draw thread. */
    @Volatile private var currentDetections: List<DetectionResult> = emptyList()
    @Volatile private var lastUpdateMs: Long = 0L

    // Reusable RectF to avoid allocations in onDraw
    private val boxRect = RectF()
    private val labelRect = RectF()

    // ── API ────────────────────────────────────────────────────────────────

    /**
     * Update the detections shown by this overlay.
     * Must be called on the **main thread** (or post to main handler before calling).
     */
    fun updateDetections(detections: List<DetectionResult>) {
        currentDetections = detections
        lastUpdateMs      = System.currentTimeMillis()
        invalidate()        // trigger onDraw
    }

    // ── Drawing ────────────────────────────────────────────────────────────

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        // Stale-result protection: suppress drawing if results are too old
        if (System.currentTimeMillis() - lastUpdateMs > DetectionState.MAX_RESULT_AGE_MS) return

        val detections = currentDetections
        if (detections.isEmpty()) return

        val vw = width.toFloat()
        val vh = height.toFloat()

        for (det in detections) {
            val colour = classColours[det.classId % classColours.size]
            boxPaint.color = colour

            // Map normalised coords → view pixels
            val left   = det.x1 * vw
            val top    = det.y1 * vh
            val right  = det.x2 * vw
            val bottom = det.y2 * vh

            boxRect.set(left, top, right, bottom)
            canvas.drawRoundRect(boxRect, 8f, 8f, boxPaint)

            // Label: "ClassName 0.87"
            val label = "${det.className} ${"%.2f".format(det.confidence)}"
            val textW = labelPaint.measureText(label)
            val textH = labelPaint.textSize
            val labelY = if (top - textH - 8f >= 0) top else bottom + textH + 8f

            labelRect.set(
                left - 2f,
                labelY - textH - 4f,
                left + textW + 6f,
                labelY + 4f
            )
            canvas.drawRoundRect(labelRect, 4f, 4f, labelBgPaint)
            labelPaint.color = colour
            canvas.drawText(label, left + 2f, labelY, labelPaint)
        }
    }
}
