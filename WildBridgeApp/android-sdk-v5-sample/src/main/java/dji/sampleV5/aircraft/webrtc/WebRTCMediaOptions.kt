package dji.sampleV5.aircraft.webrtc

/**
 * Configuration options for WebRTC media streaming.
 */
data class WebRTCMediaOptions(
    val mediaStreamId: String = "DJI_DRONE_STREAM",
    val videoTrackId: String = "DJI_VIDEO_TRACK",
    val videoResolutionWidth: Int = 1280,
    val videoResolutionHeight: Int = 720,
    val fps: Int = 30,
    val videoBitrate: Int = 2_500_000,  // 2.5 Mbps
    val videoCodec: String = "VP8"       // VP8 is widely supported
)
