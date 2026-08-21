package dji.sampleV5.aircraft.webrtc

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class WebRTCStreamMetricsTest {
    @Test
    fun compactLabelShowsWaitingAndNativeDefaults() {
        val label = WebRTCStreamMetrics(status = "idle").compactLabel()

        assertEquals(
            "WEBRTC idle out waiting req native src waiting fps 0.0/0 drop 0.0 resize 0.0ms scale fixed clients 0",
            label
        )
    }

    @Test
    fun compactLabelIncludesSaturationConfiguredFpsErrorsAndRecoveries() {
        val label = WebRTCStreamMetrics(
            sourceWidth = 1920,
            sourceHeight = 1080,
            outputWidth = 1280,
            outputHeight = 720,
            requestedWidth = 1280,
            requestedHeight = 720,
            targetFps = 30,
            outputFps = 24.25,
            droppedFps = 5.75,
            averageFrameProcessingMs = 3.49,
            processingErrors = 2,
            observerCount = 3,
            configuredFps = 24,
            saturationState = "high",
            scaleMode = "bounded",
            recoveryCount = 1,
            status = "running"
        ).compactLabel()

        assertTrue(label.contains("WEBRTC running sat high out 1280x720"))
        assertTrue(label.contains("req 1280x720 src 1920x1080"))
        assertTrue(label.contains("fps 24.3/30 cfg 24"))
        assertTrue(label.contains("drop 5.8 resize 3.5ms"))
        assertTrue(label.contains("scale bounded clients 3"))
        assertTrue(label.endsWith("err 2 fix 1"))
    }
}