package dji.sampleV5.aircraft.mavlink

/**
 * The commands the MAVLink endpoint is allowed to execute, and the only route by which an inbound
 * message can reach the aircraft.
 *
 * Until now the endpoint had no such route at all: it answered requests for messages and refused
 * everything else, which is what made it reviewable without a flight-safety argument. This
 * interface deliberately opens that door, so it is worth being explicit about how far.
 *
 * **Nothing here can move the aircraft.** The commands are payload and camera operations —
 * gimbal, zoom, recording, capture. Flight motion (takeoff, land, return, reposition, yaw,
 * altitude) is not represented in this interface at all, so no amount of malformed or malicious
 * traffic can reach it through this path. That is a deliberate first slice, not an oversight: the
 * motion commands come with their own gate and their own review.
 *
 * Implementations live in the host activity, where the DJI SDK types are, keeping this package
 * free of SDK imports for the same reason the rest of it is.
 */
internal interface MavlinkCommandSink {

    /** Absolute gimbal pitch and yaw in degrees. Moves the gimbal, never the aircraft. */
    fun setGimbalPitchYaw(pitchDeg: Float, yawDeg: Float): MavlinkCommandOutcome

    /** Absolute zoom ratio. */
    fun setCameraZoom(zoomRatio: Float): MavlinkCommandOutcome

    fun startVideoRecording(): MavlinkCommandOutcome

    fun stopVideoRecording(): MavlinkCommandOutcome

    /** Trip one shutter on the payload. Stores to the card; downloads nothing. */
    fun captureImage(): MavlinkCommandOutcome
}

/**
 * What happened, in a shape that maps straight onto `MAV_RESULT`.
 *
 * The point of a typed outcome rather than a string is that both the HTTP surface and the MAVLink
 * surface can render the same result without one of them having to parse English — which is what
 * the existing route table forces today, and the reason a ground station currently learns that a
 * command was refused by matching the text "REJECTED:".
 */
internal enum class MavlinkCommandOutcome(val mavResult: Int) {
    ACCEPTED(Mav.RESULT_ACCEPTED),

    /** The aircraft or payload cannot do this right now — no camera fitted, not connected. */
    FAILED(Mav.RESULT_FAILED),

    /** Refused on authority or safety grounds, not because it was impossible. */
    DENIED(Mav.RESULT_DENIED),

    /** Understood, but not implemented for this airframe. */
    UNSUPPORTED(Mav.RESULT_UNSUPPORTED)
}
