package dji.sampleV5.aircraft.detection

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Bitmap.Config
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.util.Log
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.support.common.FileUtil
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Asynchronous TFLite inference engine for the rhino_yolo26s model.
 *
 * ## Model contract
 *   Asset name  : rhino_yolo26s.tflite   (int8 or fp16, exported from rhino_yolo26s.pt)
 *   Labels file : rhino_yolo26s_labels.txt  (one class name per line, no index prefix)
 *   Input  shape: [1, INPUT_SIZE, INPUT_SIZE, 3] – NHWC float32 after dequant, values [0, 1]
 *   Output shape: one of:
 *     (a) [1, 4+nc, na]  – raw YOLO head (most common Ultralytics TFLite export),
 *                          where na = anchors = 33600 for 1280-pt input
 *     (b) [1, na, 6]     – post-processed (x1,y1,x2,y2,conf,cls) after built-in NMS
 *   The detector auto-detects which format is present at load time.
 *
 * ## Usage
 * ```kotlin
 * val detector = RhinoYoloDetector(context)
 * detector.detectAsync(nv21, width, height, frameNo) { results ->
 *     DetectionState.update(results)
 * }
 * ```
 *
 * ## Thread safety
 * `detectAsync` submits work to a single-thread executor so inference is
 * serialised; a busy-flag ensures old frames are dropped when inference cannot
 * keep up, preventing queue build-up that would cause UI lag.
 */
class RhinoYoloDetector(
    private val context: Context,
    private val enableNnapi: Boolean = true
) {

    companion object {
        private const val TAG = "RhinoYoloDetector"

        const val MODEL_ASSET   = "rhino_yolo26s.tflite"
        const val LABELS_ASSET  = "rhino_yolo26s_labels.txt"
        const val INPUT_SIZE    = 1280
        const val MODEL_NAME    = "rhino_yolo26s"

        /** Discard detections with confidence below this threshold post-NMS. */
        const val CONF_THRESHOLD: Float = 0.15f

        /** IoU threshold for soft-NMS / greedy NMS suppression. */
        const val IOU_THRESHOLD: Float  = 0.45f

        /** Maximum number of boxes to return per frame. */
        const val MAX_DETECTIONS: Int   = 100

        /** Number of threads to allocate to the TFLite runtime. */
        private const val TFLITE_THREADS = 4
    }

    // ── State ──────────────────────────────────────────────────────────────
    private var interpreter: Interpreter? = null
    private var labels: List<String>      = emptyList()
    private var numClasses: Int           = 1
    private var rawOutputFormat: Boolean  = true  // true = [1, 4+nc, na], false = [1, na, 6]
    private var isLoaded: Boolean         = false

    // Single-thread executor keeps GPU/DSP delegate context alive between calls.
    private val inferExecutor = Executors.newSingleThreadExecutor { r ->
        Thread(r, "RhinoYoloInference").also { it.isDaemon = true }
    }

    // Busy flag: if the previous inference is still running, skip the new frame
    // rather than enqueue, so the overlay never shows boxes that are >1 frame stale.
    private val isBusy = AtomicBoolean(false)

    // Reusable pre-allocated buffer: 1 × INPUT_SIZE × INPUT_SIZE × 3 × 4 bytes
    private val inputBuffer: ByteBuffer = ByteBuffer
        .allocateDirect(1 * INPUT_SIZE * INPUT_SIZE * 3 * 4)
        .order(ByteOrder.nativeOrder())

    init {
        try {
            loadModel()
        } catch (e: Exception) {
            Log.e(TAG, "Failed to load model on init (will retry on first call): ${e.message}", e)
        }
    }

    // ── Public API ─────────────────────────────────────────────────────────

    /**
     * Submit an inference job for the supplied NV21 frame.
     * If a previous job is still executing the frame is silently discarded to
     * prevent latency build-up.
     *
     * @param nv21        NV21-encoded camera frame (copied, safe to reuse after return)
     * @param srcWidth    Frame width in pixels
     * @param srcHeight   Frame height in pixels
     * @param frameNumber Monotonic frame counter (echoed in each [DetectionResult])
     * @param callback    Invoked on the inference thread with the list of detections
     *                    (may be empty if nothing is detected above the threshold)
     */
    fun detectAsync(
        nv21: ByteArray,
        srcWidth: Int,
        srcHeight: Int,
        frameNumber: Long,
        callback: (List<DetectionResult>) -> Unit
    ) {
        if (!isBusy.compareAndSet(false, true)) {
            Log.v(TAG, "Inference busy – skipping frame $frameNumber")
            return
        }

        // Capture a snapshot of the input data for the lambda
        val inputCopy = if (nv21.isEmpty()) return.also { isBusy.set(false) } else nv21

        inferExecutor.submit {
            try {
                if (!isLoaded) loadModel()

                val t0 = System.currentTimeMillis()
                val bitmap = nv21ToBitmap(inputCopy, srcWidth, srcHeight)
                    ?: return@submit

                val results = runInference(bitmap, frameNumber, t0)
                bitmap.recycle()
                callback(results)
            } catch (e: Exception) {
                Log.e(TAG, "Inference error on frame $frameNumber: ${e.message}", e)
                callback(emptyList())
            } finally {
                isBusy.set(false)
            }
        }
    }

    /**
     * Async inference entry-point for already-decoded bitmaps.
     * Used by local-video test activity where frames come from
     * MediaMetadataRetriever.
     */
    fun detectBitmapAsync(
        bitmap: Bitmap,
        frameNumber: Long,
        callback: (List<DetectionResult>) -> Unit
    ) {
        if (!isBusy.compareAndSet(false, true)) {
            Log.v(TAG, "Inference busy – skipping bitmap frame $frameNumber")
            return
        }
        inferExecutor.submit {
            try {
                if (!isLoaded) loadModel()
                val t0 = System.currentTimeMillis()

                val snapshot = bitmap.copy(Bitmap.Config.ARGB_8888, false)
                    ?: run {
                        callback(emptyList())
                        return@submit
                    }

                val inputBitmap = if (snapshot.width == INPUT_SIZE && snapshot.height == INPUT_SIZE) {
                    snapshot
                } else {
                    val scaled = Bitmap.createScaledBitmap(snapshot, INPUT_SIZE, INPUT_SIZE, true)
                    snapshot.recycle()
                    scaled
                }

                val results = runInference(inputBitmap, frameNumber, t0)
                inputBitmap.recycle()
                callback(results)
            } catch (e: Exception) {
                Log.e(TAG, "Bitmap inference error on frame $frameNumber: ${e.message}", e)
                callback(emptyList())
            } finally {
                isBusy.set(false)
            }
        }
    }

    /**
     * Async inference for caller-managed input bitmap.
     *
     * Caller must provide an ARGB_8888 bitmap of size INPUT_SIZE x INPUT_SIZE,
     * and must not mutate it until callback is invoked.
     */
    fun detectPreparedBitmapAsync(
        preparedBitmap: Bitmap,
        frameNumber: Long,
        callback: (List<DetectionResult>) -> Unit
    ) {
        if (!isBusy.compareAndSet(false, true)) {
            Log.v(TAG, "Inference busy – skipping prepared bitmap frame $frameNumber")
            return
        }

        inferExecutor.submit {
            try {
                if (!isLoaded) loadModel()

                if (preparedBitmap.width != INPUT_SIZE || preparedBitmap.height != INPUT_SIZE || preparedBitmap.config != Config.ARGB_8888) {
                    Log.w(TAG, "Prepared bitmap mismatch; expected ${INPUT_SIZE}x${INPUT_SIZE} ARGB_8888")
                    callback(emptyList())
                    return@submit
                }

                val t0 = System.currentTimeMillis()
                val results = runInference(preparedBitmap, frameNumber, t0)
                callback(results)
            } catch (e: Exception) {
                Log.e(TAG, "Prepared bitmap inference error on frame $frameNumber: ${e.message}", e)
                callback(emptyList())
            } finally {
                isBusy.set(false)
            }
        }
    }

    fun canAcceptFrame(): Boolean = !isBusy.get()

    fun release() {
        interpreter?.close()
        interpreter = null
        isLoaded    = false
        inferExecutor.shutdown()
        Log.d(TAG, "Released")
    }

    // ── Model loading ──────────────────────────────────────────────────────

    private fun loadModel() {
        Log.d(TAG, "Loading model: $MODEL_ASSET")

        // ── labels ──────────────────────────────────────────────────────
        labels = try {
            context.assets.open(LABELS_ASSET)
                .bufferedReader()
                .readLines()
                .map { it.trim() }
                .filter { it.isNotEmpty() }
        } catch (e: Exception) {
            Log.w(TAG, "Labels file not found – falling back to single class 'animal'")
            listOf("animal")
        }
        numClasses = labels.size
        Log.d(TAG, "Loaded ${labels.size} class labels: ${labels.take(5)}")

        // ── interpreter ─────────────────────────────────────────────────
        val modelBuffer = FileUtil.loadMappedFile(context, MODEL_ASSET)
        val options = Interpreter.Options().apply {
            numThreads = TFLITE_THREADS
            useNNAPI   = enableNnapi
        }
        interpreter = Interpreter(modelBuffer, options)

        // ── determine output format ──────────────────────────────────────
        val outputTensor = interpreter!!.getOutputTensor(0)
        val outShape = outputTensor.shape()
        Log.d(TAG, "Output tensor shape: ${outShape.toList()}")

        rawOutputFormat = when {
            outShape.size == 3 && outShape[1] == (4 + numClasses) -> {
                // [1, 4+nc, na]  – raw YOLO head
                Log.d(TAG, "Using raw YOLO output format [1, ${4 + numClasses}, ${outShape[2]}]")
                true
            }
            outShape.size == 3 && outShape[2] == 6 -> {
                // [1, na, 6]  – post-processed with built-in NMS
                Log.d(TAG, "Using post-processed output format [1, ${outShape[1]}, 6]")
                false
            }
            else -> {
                Log.w(TAG, "Unrecognised output shape ${outShape.toList()} – assuming raw format")
                true
            }
        }

        isLoaded = true
        Log.i(TAG, "Model loaded successfully. Input: ${INPUT_SIZE}×${INPUT_SIZE}, classes=$numClasses")
    }

    // ── Preprocessing ──────────────────────────────────────────────────────

    private fun nv21ToBitmap(nv21: ByteArray, width: Int, height: Int): Bitmap? {
        return try {
            val yuvImage = YuvImage(nv21, ImageFormat.NV21, width, height, null)
            val out = ByteArrayOutputStream()
            yuvImage.compressToJpeg(Rect(0, 0, width, height), 85, out)
            val raw = out.toByteArray()
            val full = BitmapFactory.decodeByteArray(raw, 0, raw.size) ?: return null

            // Scale to INPUT_SIZE × INPUT_SIZE in-place (letterbox is not needed here
            // because YOLO's own normalisation handles aspect ratio during training)
            val scaled = if (full.width == INPUT_SIZE && full.height == INPUT_SIZE) {
                full
            } else {
                val s = Bitmap.createScaledBitmap(full, INPUT_SIZE, INPUT_SIZE, true)
                full.recycle()
                s
            }
            scaled
        } catch (e: Exception) {
            Log.e(TAG, "NV21 → Bitmap conversion failed: ${e.message}")
            null
        }
    }

    private fun bitmapToFloatBuffer(bitmap: Bitmap): ByteBuffer {
        inputBuffer.rewind()
        val pixels = IntArray(INPUT_SIZE * INPUT_SIZE)
        bitmap.getPixels(pixels, 0, INPUT_SIZE, 0, 0, INPUT_SIZE, INPUT_SIZE)
        for (pixel in pixels) {
            inputBuffer.putFloat(((pixel shr 16) and 0xFF) / 255.0f)   // R
            inputBuffer.putFloat(((pixel shr 8)  and 0xFF) / 255.0f)   // G
            inputBuffer.putFloat((pixel          and 0xFF) / 255.0f)   // B
        }
        inputBuffer.rewind()
        return inputBuffer
    }

    // ── Inference & post-processing ────────────────────────────────────────

    private fun runInference(
        bitmap: Bitmap,
        frameNumber: Long,
        startTimeMs: Long
    ): List<DetectionResult> {
        val interp = interpreter ?: return emptyList()

        val input = bitmapToFloatBuffer(bitmap)

        return if (rawOutputFormat) {
            // [1, 4+nc, na]
            val na = interp.getOutputTensor(0).shape()[2]
            val output = Array(1) { Array(4 + numClasses) { FloatArray(na) } }
            interp.run(input, output)
            val inferMs = System.currentTimeMillis() - startTimeMs
            decodeRawOutput(output[0], na, inferMs, frameNumber)
        } else {
            // [1, na, 6]
            val na = interp.getOutputTensor(0).shape()[1]
            val output = Array(1) { Array(na) { FloatArray(6) } }
            interp.run(input, output)
            val inferMs = System.currentTimeMillis() - startTimeMs
            decodePostProcessedOutput(output[0], na, inferMs, frameNumber)
        }
    }

    /** Decode raw YOLO output [4+nc, na] → detections. */
    private fun decodeRawOutput(
        output: Array<FloatArray>,
        na: Int,
        inferMs: Long,
        frameNumber: Long
    ): List<DetectionResult> {
        val candidates = mutableListOf<FloatArray>() // [x, y, w, h, maxConf, classId]

        for (i in 0 until na) {
            // output[0..3][i] = cx, cy, w, h (normalised 0-1)
            val cx = output[0][i]
            val cy = output[1][i]
            val w  = output[2][i]
            val h  = output[3][i]

            // Find best class
            var maxConf = 0f
            var bestCls = 0
            for (c in 0 until numClasses) {
                val conf = output[4 + c][i]
                if (conf > maxConf) { maxConf = conf; bestCls = c }
            }

            if (maxConf >= CONF_THRESHOLD) {
                candidates.add(floatArrayOf(
                    cx - w / 2f, cy - h / 2f,   // x1, y1
                    cx + w / 2f, cy + h / 2f,   // x2, y2
                    maxConf, bestCls.toFloat()
                ))
            }
        }

        return applyNmsAndBuild(candidates, inferMs, frameNumber)
    }

    /** Decode post-processed YOLO output [na, 6] → detections. */
    private fun decodePostProcessedOutput(
        output: Array<FloatArray>,
        na: Int,
        inferMs: Long,
        frameNumber: Long
    ): List<DetectionResult> {
        val candidates = mutableListOf<FloatArray>()
        for (i in 0 until na) {
            val row = output[i]
            if (row.size < 6) continue
            val conf = row[4]
            if (conf >= CONF_THRESHOLD) {
                candidates.add(floatArrayOf(row[0], row[1], row[2], row[3], conf, row[5]))
            }
        }
        return applyNmsAndBuild(candidates, inferMs, frameNumber)
    }

    /**
     * Greedy IoU-based NMS then build [DetectionResult] objects.
     * Candidate format: [x1, y1, x2, y2, conf, classId]
     */
    private fun applyNmsAndBuild(
        candidates: MutableList<FloatArray>,
        inferMs: Long,
        frameNumber: Long
    ): List<DetectionResult> {
        // Sort descending by confidence
        candidates.sortByDescending { it[4] }

        val kept = mutableListOf<FloatArray>()
        val suppressed = BooleanArray(candidates.size)

        for (i in candidates.indices) {
            if (suppressed[i]) continue
            kept.add(candidates[i])
            if (kept.size >= MAX_DETECTIONS) break

            for (j in (i + 1) until candidates.size) {
                if (suppressed[j]) continue
                if (iou(candidates[i], candidates[j]) > IOU_THRESHOLD) {
                    suppressed[j] = true
                }
            }
        }

        val results = kept.map { row ->
            val classId = row[5].toInt().coerceIn(0, labels.size - 1)
            DetectionResult(
                classId     = classId,
                className   = labels.getOrElse(classId) { "cls_$classId" },
                confidence  = row[4],
                x1 = row[0].coerceIn(0f, 1f),
                y1 = row[1].coerceIn(0f, 1f),
                x2 = row[2].coerceIn(0f, 1f),
                y2 = row[3].coerceIn(0f, 1f),
                inferenceMs = inferMs,
                modelName   = MODEL_NAME,
                frameNumber = frameNumber
            )
        }

        if (results.isNotEmpty()) {
            Log.d(TAG, "Frame $frameNumber: ${results.size} detection(s) in ${inferMs}ms – " +
                    results.joinToString { "${it.className} ${"%.2f".format(it.confidence)}" })
        }
        return results
    }

    private fun iou(a: FloatArray, b: FloatArray): Float {
        val interX1 = maxOf(a[0], b[0])
        val interY1 = maxOf(a[1], b[1])
        val interX2 = minOf(a[2], b[2])
        val interY2 = minOf(a[3], b[3])

        val interArea = maxOf(0f, interX2 - interX1) * maxOf(0f, interY2 - interY1)
        if (interArea == 0f) return 0f

        val areaA = (a[2] - a[0]) * (a[3] - a[1])
        val areaB = (b[2] - b[0]) * (b[3] - b[1])
        return interArea / (areaA + areaB - interArea)
    }
}
