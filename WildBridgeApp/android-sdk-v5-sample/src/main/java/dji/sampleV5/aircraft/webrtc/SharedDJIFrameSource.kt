package dji.sampleV5.aircraft.webrtc

import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import org.webrtc.CapturerObserver
import org.webrtc.NV21Buffer
import org.webrtc.VideoFrame
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * Shared video frame source that registers a single DJI CameraFrameListener,
 * scales once, and broadcasts the resulting VideoFrame to all registered
 * WebRTC CapturerObservers.  This eliminates duplicate NV21→scale→encode work
 * when multiple viewers are connected.
 */
class SharedDJIFrameSource(
    private val cameraIndex: ComponentIndexType,
    private val droneName: String
) {
    companion object {
        private const val TAG = "SharedDJIFrameSource"
    }

    private val observers = ConcurrentHashMap<String, CapturerObserver>()
    private val metadataListeners = ConcurrentHashMap<String, DJIV5VideoCapturer.FrameMetadataListener>()
    private val isCapturing = AtomicBoolean(false)

    @Volatile var targetWidth: Int = DJIV5VideoCapturer.FULL_HD_WIDTH
    @Volatile var targetHeight: Int = DJIV5VideoCapturer.FULL_HD_HEIGHT
    @Volatile private var scaleToTarget: Boolean = true
    @Volatile private var targetFps: Int = 30
    @Volatile private var frameIntervalNs: Long = 1_000_000_000L / 30L
    private val lastSentTimestampNs = AtomicLong(0L)
    private val frameCounter = AtomicLong(0)
    private var lastSourceWidth = 0
    private var lastSourceHeight = 0

    private val cameraStreamManager: ICameraStreamManager by lazy {
        MediaDataCenter.getInstance().cameraStreamManager
    }

    private val frameListener = object : ICameraStreamManager.CameraFrameListener {
        override fun onFrame(
            frameData: ByteArray,
            offset: Int,
            length: Int,
            width: Int,
            height: Int,
            format: ICameraStreamManager.FrameFormat
        ) {
            if (!isCapturing.get()) return
            val currentObservers = observers.values.toList()
            if (currentObservers.isEmpty()) return

            try {
                val timestampNs = System.nanoTime()

                val previousSent = lastSentTimestampNs.get()
                if (previousSent != 0L && (timestampNs - previousSent) < frameIntervalNs) {
                    return
                }
                lastSentTimestampNs.set(timestampNs)

                if (width != lastSourceWidth || height != lastSourceHeight) {
                    lastSourceWidth = width
                    lastSourceHeight = height
                    Log.d(TAG, "Source: ${width}x${height}, Target: ${targetWidth}x${targetHeight}, Scale: $scaleToTarget")
                }

                val frameNumber = frameCounter.incrementAndGet()

                val outputWidth = if (scaleToTarget) targetWidth else width
                val outputHeight = if (scaleToTarget) targetHeight else height

                // Capture telemetry once and broadcast to all metadata listeners
                val currentListeners = metadataListeners.values.toList()
                if (currentListeners.isNotEmpty()) {
                    val metadata = TelemetryProvider.captureMetadata(
                        frameNumber = frameNumber,
                        timestampNs = timestampNs,
                        frameWidth = outputWidth,
                        frameHeight = outputHeight,
                        droneName = droneName
                    )
                    currentListeners.forEach { it.onFrameMetadata(metadata) }
                }

                // Create NV21 buffer and scale ONCE
                val buffer = NV21Buffer(frameData, width, height, null)

                val needsScale = scaleToTarget && (width != targetWidth || height != targetHeight)
                val outputBuffer = if (needsScale) {
                    val scaled = buffer.cropAndScale(0, 0, width, height, targetWidth, targetHeight)
                    buffer.release()
                    scaled
                } else {
                    buffer
                }

                // Broadcast the same VideoFrame to every observer.
                // Retain once per extra observer; the first consumer uses the initial ref.
                val videoFrame = VideoFrame(outputBuffer, 0, timestampNs)
                val extra = currentObservers.size - 1
                repeat(extra) { videoFrame.retain() }
                currentObservers.forEach { it.onFrameCaptured(videoFrame) }
                videoFrame.release()

            } catch (e: Exception) {
                Log.e(TAG, "Error processing frame: ${e.message}", e)
            }
        }
    }

    // ---- Client management ----

    fun registerObserver(clientId: String, observer: CapturerObserver) {
        observers[clientId] = observer
        Log.d(TAG, "Observer registered: $clientId (total: ${observers.size})")
    }

    fun setMetadataListener(clientId: String, listener: DJIV5VideoCapturer.FrameMetadataListener?) {
        if (listener != null) {
            metadataListeners[clientId] = listener
        } else {
            metadataListeners.remove(clientId)
        }
    }

    /**
     * Start capturing if not already started. If already capturing, the
     * new client immediately begins receiving frames.
     */
    fun startClient(clientId: String, width: Int, height: Int, fps: Int) {
        // Use the first client's requested settings
        if (isCapturing.compareAndSet(false, true)) {
            targetWidth = width
            targetHeight = height
            targetFps = fps.coerceAtLeast(1)
            frameIntervalNs = 1_000_000_000L / targetFps.toLong()
            lastSentTimestampNs.set(0L)
            Log.d(TAG, "Starting shared capture: ${targetWidth}x${targetHeight}@${targetFps}fps")
            cameraStreamManager.addFrameListener(
                cameraIndex,
                ICameraStreamManager.FrameFormat.NV21,
                frameListener
            )
        }
        observers[clientId]?.onCapturerStarted(true)
    }

    fun stopClient(clientId: String) {
        observers[clientId]?.onCapturerStopped()
        observers.remove(clientId)
        metadataListeners.remove(clientId)
        Log.d(TAG, "Client removed: $clientId (remaining: ${observers.size})")
        if (observers.isEmpty() && isCapturing.compareAndSet(true, false)) {
            Log.d(TAG, "No observers left – stopping shared capture")
            cameraStreamManager.removeFrameListener(frameListener)
        }
    }

    fun changeResolution(width: Int, height: Int) {
        Log.d(TAG, "Changing target resolution: ${targetWidth}x${targetHeight} -> ${width}x${height}")
        targetWidth = width
        targetHeight = height
    }

    fun dispose() {
        if (isCapturing.compareAndSet(true, false)) {
            cameraStreamManager.removeFrameListener(frameListener)
        }
        observers.clear()
        metadataListeners.clear()
        Log.d(TAG, "SharedDJIFrameSource disposed")
    }
}
