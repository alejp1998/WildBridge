package dji.sampleV5.aircraft.edge

import android.content.Context
import android.media.Image
import android.net.Uri
import android.util.Log
import dji.sampleV5.aircraft.webrtc.SharedDJIFrameSource
import dji.v5.ux.detection.DetectedTarget
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

class EdgeDetectionController(
    context: Context,
    private val modelUri: Uri?,
    private val labels: List<String>,
    private val sourceLabel: String,
    private val confidenceThreshold: Float = 0.25f,
    private val onTargets: (List<DetectedTarget>) -> Unit,
    private val onMetrics: (EdgeDetectionMetrics) -> Unit = {}
) : SharedDJIFrameSource.EdgeDetectionFrameListener {

    companion object {
        private const val TAG = "EdgeDetectionController"
        private const val TARGET_FPS = 5
        private const val FRAME_INTERVAL_NS = 1_000_000_000L / TARGET_FPS
    }

    private val appContext = context.applicationContext
    private val executor = Executors.newSingleThreadExecutor()
    private val busy = AtomicBoolean(false)
    @Volatile private var detector: YoloTfliteDetector? = null
    @Volatile private var running = false
    @Volatile private var lastInferenceNs = 0L
    @Volatile private var status = "loading"
    private var receivedInWindow = 0L
    private var inferredInWindow = 0L
    private var droppedInWindow = 0L
    private var inferenceTimeNsInWindow = 0L
    private var lastMetricsAtNs = System.nanoTime()

    data class EdgeDetectionMetrics(
        val status: String = "off",
        val source: String = "dji",
        val targetFps: Int = TARGET_FPS,
        val inputFps: Double = 0.0,
        val inferenceFps: Double = 0.0,
        val droppedFps: Double = 0.0,
        val averageInferenceMs: Double = 0.0,
        val targetCount: Int = 0,
        val confidenceThreshold: Float = 0.25f,
        val lastError: String? = null
    ) {
        fun compactLabel(): String {
            val errorLabel = lastError?.takeIf { it.isNotBlank() }?.let { " err" } ?: ""
            return "EDGE $status src $source th ${(confidenceThreshold * 100).toInt()} fps ${inferenceFps.format1()}/$targetFps in ${inputFps.format1()} drop ${droppedFps.format1()} infer ${averageInferenceMs.format1()}ms targets $targetCount$errorLabel"
        }

        private fun Double.format1(): String = String.format(java.util.Locale.US, "%.1f", this)
    }

    fun start() {
        if (running) return
        val selectedModelUri = modelUri
        if (selectedModelUri == null) {
            status = "no-model"
            emitImmediateMetrics(targetCount = 0, lastError = "No model selected")
            onTargets(emptyList())
            return
        }
        running = true
        status = "loading"
        emitImmediateMetrics(targetCount = 0)
        executor.execute {
            runCatching {
                detector = YoloTfliteDetector.fromUri(appContext, selectedModelUri, labels, confidenceThreshold)
                status = "ready"
                emitImmediateMetrics(targetCount = 0)
                Log.i(TAG, "Edge detector loaded: $selectedModelUri")
            }.onFailure {
                running = false
                status = "error"
                Log.e(TAG, "Failed to load edge detector: ${it.message}", it)
                emitImmediateMetrics(targetCount = 0, lastError = it.message)
                onTargets(emptyList())
            }
        }
    }

    fun stop() {
        running = false
        busy.set(false)
        executor.execute {
            detector?.close()
            detector = null
            status = "off"
            emitImmediateMetrics(targetCount = 0)
            onTargets(emptyList())
        }
    }

    fun dispose() {
        stop()
        executor.shutdownNow()
    }

    override fun onNv21Frame(frameData: ByteArray, offset: Int, length: Int, width: Int, height: Int, timestampNs: Long) {
        if (!running || detector == null) return
        receivedInWindow++
        if (timestampNs - lastInferenceNs < FRAME_INTERVAL_NS) {
            droppedInWindow++
            maybeEmitWindowMetrics(timestampNs, targetCount = 0)
            return
        }
        if (!busy.compareAndSet(false, true)) {
            droppedInWindow++
            maybeEmitWindowMetrics(timestampNs, targetCount = 0)
            return
        }
        lastInferenceNs = timestampNs

        val frameCopy = frameData.copyOfRange(offset, offset + length)
        executor.execute {
            val inferenceStartNs = System.nanoTime()
            var targetCount = 0
            try {
                val targets = detector?.detectNv21(frameCopy, 0, frameCopy.size, width, height).orEmpty()
                targetCount = targets.size
                inferredInWindow++
                inferenceTimeNsInWindow += System.nanoTime() - inferenceStartNs
                status = "running"
                if (running) onTargets(targets)
            } catch (e: Exception) {
                status = "error"
                Log.e(TAG, "Edge inference failed: ${e.message}", e)
                emitImmediateMetrics(targetCount = targetCount, lastError = e.message)
            } finally {
                maybeEmitWindowMetrics(System.nanoTime(), targetCount)
                busy.set(false)
            }
        }
    }

    fun onYuv420Image(image: Image, timestampNs: Long, onComplete: () -> Unit = {}) {
        if (!running || detector == null) {
            image.close()
            onComplete()
            return
        }
        receivedInWindow++
        if (timestampNs - lastInferenceNs < FRAME_INTERVAL_NS) {
            droppedInWindow++
            maybeEmitWindowMetrics(timestampNs, targetCount = 0)
            image.close()
            onComplete()
            return
        }
        if (!busy.compareAndSet(false, true)) {
            droppedInWindow++
            maybeEmitWindowMetrics(timestampNs, targetCount = 0)
            image.close()
            onComplete()
            return
        }
        lastInferenceNs = timestampNs

        executor.execute {
            val inferenceStartNs = System.nanoTime()
            var targetCount = 0
            try {
                val targets = detector?.detectYuv420(image).orEmpty()
                targetCount = targets.size
                inferredInWindow++
                inferenceTimeNsInWindow += System.nanoTime() - inferenceStartNs
                status = "running"
                if (running) onTargets(targets)
            } catch (e: Exception) {
                status = "error"
                Log.e(TAG, "Edge YUV inference failed: ${e.message}", e)
                emitImmediateMetrics(targetCount = targetCount, lastError = e.message)
            } finally {
                image.close()
                maybeEmitWindowMetrics(System.nanoTime(), targetCount)
                busy.set(false)
                onComplete()
            }
        }
    }

    private fun maybeEmitWindowMetrics(nowNs: Long, targetCount: Int) {
        val elapsedNs = nowNs - lastMetricsAtNs
        if (elapsedNs < 1_000_000_000L) return
        val elapsedSeconds = elapsedNs / 1_000_000_000.0
        val inferred = inferredInWindow
        val averageInferenceMs = if (inferred > 0) inferenceTimeNsInWindow / inferred / 1_000_000.0 else 0.0
        onMetrics(
            EdgeDetectionMetrics(
                status = status,
                source = sourceLabel,
                inputFps = receivedInWindow / elapsedSeconds,
                inferenceFps = inferred / elapsedSeconds,
                droppedFps = droppedInWindow / elapsedSeconds,
                averageInferenceMs = averageInferenceMs,
                targetCount = targetCount,
                confidenceThreshold = confidenceThreshold,
            )
        )
        receivedInWindow = 0L
        inferredInWindow = 0L
        droppedInWindow = 0L
        inferenceTimeNsInWindow = 0L
        lastMetricsAtNs = nowNs
    }

    private fun emitImmediateMetrics(targetCount: Int, lastError: String? = null) {
        onMetrics(
            EdgeDetectionMetrics(
                status = status,
                source = sourceLabel,
                targetCount = targetCount,
                confidenceThreshold = confidenceThreshold,
                lastError = lastError
            )
        )
    }
}