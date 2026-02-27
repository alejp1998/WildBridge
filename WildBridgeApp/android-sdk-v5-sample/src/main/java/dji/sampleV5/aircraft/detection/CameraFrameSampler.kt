package dji.sampleV5.aircraft.detection

import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * Lightweight camera tap that samples NV21 frames from the DJI stream at a
 * reduced rate (every [sampleEveryN] frames) and forwards them to a callback.
 *
 * This is intentionally separate from DJIV5VideoCapturer so that: 1) the
 * WebRTC video pipeline is unaffected, and 2) sampling can be started/stopped
 * independently of streaming.
 *
 * Typical use:
 * ```
 * sampler = CameraFrameSampler(sampleEveryN = 5)
 * sampler.callback = { nv21, w, h, frameNo -> runInference(nv21, w, h, frameNo) }
 * sampler.start()    // called when detection is toggled ON
 * sampler.stop()     // called when detection is toggled OFF
 * ```
 */
class CameraFrameSampler(
    private val cameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    val sampleEveryN: Int = 5
) {
    companion object { private const val TAG = "CameraFrameSampler" }

    /** Called on the DJI capture thread with a *copy* of the NV21 plane. */
    var callback: ((nv21: ByteArray, width: Int, height: Int, frameNumber: Long) -> Unit)? = null

    private val isRunning = AtomicBoolean(false)
    private val frameCounter = AtomicLong(0)

    private val streamListener = object : ICameraStreamManager.CameraFrameListener {
        override fun onFrame(
            frameData: ByteArray,
            offset: Int,
            length: Int,
            width: Int,
            height: Int,
            format: ICameraStreamManager.FrameFormat
        ) {
            val n = frameCounter.incrementAndGet()
            if (n % sampleEveryN == 0L) {
                // Copy the relevant slice – frameData may be a reused ring buffer.
                val copy = frameData.copyOfRange(offset, offset + length)
                try {
                    callback?.invoke(copy, width, height, n)
                } catch (e: Exception) {
                    Log.e(TAG, "Frame callback error: ${e.message}", e)
                }
            }
        }
    }

    fun start() {
        if (!isRunning.compareAndSet(false, true)) {
            Log.d(TAG, "Already running")
            return
        }
        frameCounter.set(0)
        MediaDataCenter.getInstance().cameraStreamManager.addFrameListener(
            cameraIndex,
            ICameraStreamManager.FrameFormat.NV21,
            streamListener
        )
        Log.d(TAG, "Started (every ${sampleEveryN} frames)")
    }

    fun stop() {
        if (!isRunning.compareAndSet(true, false)) {
            Log.d(TAG, "Already stopped")
            return
        }
        MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(streamListener)
        Log.d(TAG, "Stopped")
    }

    val running: Boolean get() = isRunning.get()
}
