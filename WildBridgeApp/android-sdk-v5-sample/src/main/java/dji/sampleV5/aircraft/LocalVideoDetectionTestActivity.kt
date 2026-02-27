package dji.sampleV5.aircraft

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Rect
import android.media.MediaMetadataRetriever
import android.net.Uri
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.ImageView
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import dji.sampleV5.aircraft.detection.DetectionOverlayView
import dji.sampleV5.aircraft.detection.DetectionState
import dji.sampleV5.aircraft.detection.RhinoYoloDetector

/**
 * Manual local-video tester for edge detection model.
 *
 * Purpose:
 * - Validate that model + labels are correctly bundled on device.
 * - Validate overlay drawing logic without requiring live DJI camera feed.
 *
 * Behavior:
 * - User picks a local video file from device storage.
 * - The activity samples frames at fixed intervals (default 200 ms).
 * - Each sampled frame is sent to RhinoYoloDetector.
 * - Detections are drawn on the overlay above the frame preview.
 */
class LocalVideoDetectionTestActivity : AppCompatActivity() {

    companion object {
        private const val FRAME_STEP_MS = 200L
    }

    private lateinit var imageFrame: ImageView
    private lateinit var overlay: DetectionOverlayView
    private lateinit var statusText: TextView
    private lateinit var toggleButton: Button

    private var detector: RhinoYoloDetector? = null
    private var retriever: MediaMetadataRetriever? = null
    private var selectedVideoUri: Uri? = null

    private var videoDurationMs: Long = 0L
    private var currentTimeMs: Long = 0L
    private var running = false
    private var frameCounter = 0L
    private var lastDetectionCallbackMs: Long = 0L
    private var smoothedDetectionFps: Float = 0f
    private var lastPreviewBitmap: Bitmap? = null
    private var inferenceBitmap: Bitmap? = null
    private var inferenceCanvas: Canvas? = null
    private val inferenceDstRect = Rect(0, 0, RhinoYoloDetector.INPUT_SIZE, RhinoYoloDetector.INPUT_SIZE)

    private val mainHandler = Handler(Looper.getMainLooper())

    private val pickVideoLauncher = registerForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        if (uri == null) return@registerForActivityResult
        try {
            contentResolver.takePersistableUriPermission(uri, android.content.Intent.FLAG_GRANT_READ_URI_PERMISSION)
        } catch (_: SecurityException) {
        }
        loadVideo(uri)
    }

    private val frameLoop = object : Runnable {
        override fun run() {
            if (!running) return
            val r = retriever ?: return

            if (currentTimeMs > videoDurationMs) {
                running = false
                toggleButton.text = "Start Test"
                statusText.text = "Finished"
                return
            }

            val bitmap = try {
                r.getFrameAtTime(currentTimeMs * 1000, MediaMetadataRetriever.OPTION_CLOSEST)
            } catch (_: Exception) {
                null
            }

            if (bitmap != null) {
                lastPreviewBitmap?.let { old ->
                    if (old != bitmap && !old.isRecycled) {
                        old.recycle()
                    }
                }
                lastPreviewBitmap = bitmap
                imageFrame.setImageBitmap(bitmap)
                runDetection(bitmap)
            }

            currentTimeMs += FRAME_STEP_MS
            mainHandler.postDelayed(this, FRAME_STEP_MS)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_local_video_detection_test)

        imageFrame = findViewById(R.id.image_frame)
        overlay = findViewById(R.id.view_local_detection_overlay)
        statusText = findViewById(R.id.txt_status)
        toggleButton = findViewById(R.id.btn_toggle_test)

        detector = RhinoYoloDetector(this, enableNnapi = true)
        inferenceBitmap = Bitmap.createBitmap(
            RhinoYoloDetector.INPUT_SIZE,
            RhinoYoloDetector.INPUT_SIZE,
            Bitmap.Config.ARGB_8888
        )
        inferenceCanvas = Canvas(inferenceBitmap!!)

        val hasModel = hasAsset(RhinoYoloDetector.MODEL_ASSET)
        if (!hasModel) {
            statusText.text = "Missing model asset: ${RhinoYoloDetector.MODEL_ASSET}"
            toggleButton.isEnabled = false
            Toast.makeText(this, "Model not found in assets", Toast.LENGTH_LONG).show()
        }

        findViewById<Button>(R.id.btn_pick_video).setOnClickListener {
            pickVideoLauncher.launch(arrayOf("video/*"))
        }

        toggleButton.setOnClickListener {
            if (running) stopLoop() else startLoop()
        }
    }

    private fun loadVideo(uri: Uri) {
        stopLoop()
        selectedVideoUri = uri
        retriever?.release()

        val newRetriever = MediaMetadataRetriever()
        try {
            newRetriever.setDataSource(this, uri)
        } catch (e: Exception) {
            newRetriever.release()
            statusText.text = "Failed to open video"
            Toast.makeText(this, "Failed to open selected video", Toast.LENGTH_SHORT).show()
            return
        }

        retriever = newRetriever
        videoDurationMs = (newRetriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION)?.toLongOrNull() ?: 0L)
        currentTimeMs = 0L
        frameCounter = 0L
        lastDetectionCallbackMs = 0L
        smoothedDetectionFps = 0f
        statusText.text = "Video loaded (${videoDurationMs} ms)"
    }

    private fun startLoop() {
        if (selectedVideoUri == null || retriever == null) {
            Toast.makeText(this, "Pick a video first", Toast.LENGTH_SHORT).show()
            return
        }
        running = true
        toggleButton.text = "Stop Test"
        statusText.text = "Running local detection test..."
        mainHandler.removeCallbacks(frameLoop)
        mainHandler.post(frameLoop)
    }

    private fun stopLoop() {
        running = false
        toggleButton.text = "Start Test"
        mainHandler.removeCallbacks(frameLoop)
        imageFrame.setImageDrawable(null)
        lastPreviewBitmap?.let {
            if (!it.isRecycled) {
                it.recycle()
            }
        }
        lastPreviewBitmap = null
    }

    private fun runDetection(bitmap: Bitmap) {
        val currentDetector = detector ?: return
        val preparedBitmap = inferenceBitmap ?: return
        val preparedCanvas = inferenceCanvas ?: return
        if (!currentDetector.canAcceptFrame()) {
            return
        }

        preparedCanvas.drawBitmap(bitmap, null, inferenceDstRect, null)

        frameCounter += 1
        currentDetector.detectPreparedBitmapAsync(preparedBitmap, frameCounter) { detections ->
            DetectionState.update(detections)
            val now = System.currentTimeMillis()
            val instantaneousFps = if (lastDetectionCallbackMs > 0L) {
                val dtMs = (now - lastDetectionCallbackMs).coerceAtLeast(1L)
                1000f / dtMs.toFloat()
            } else {
                0f
            }
            lastDetectionCallbackMs = now
            smoothedDetectionFps = if (smoothedDetectionFps == 0f) {
                instantaneousFps
            } else {
                (smoothedDetectionFps * 0.8f) + (instantaneousFps * 0.2f)
            }

            val inferenceMs = detections.firstOrNull()?.inferenceMs ?: -1L
            val inferenceLabel = if (inferenceMs >= 0L) "${inferenceMs}ms" else "--"
            mainHandler.post {
                overlay.updateDetections(detections)
                statusText.text = "t=${currentTimeMs}ms  frame=${frameCounter}  det=${detections.size}  fps=${"%.1f".format(smoothedDetectionFps)}  inf=${inferenceLabel}"
            }
        }
    }

    private fun hasAsset(name: String): Boolean {
        return try {
            assets.open(name).use { true }
        } catch (_: Exception) {
            false
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopLoop()
        retriever?.release()
        retriever = null
        inferenceCanvas = null
        inferenceBitmap?.let {
            if (!it.isRecycled) {
                it.recycle()
            }
        }
        inferenceBitmap = null
        detector?.release()
        detector = null
    }
}
