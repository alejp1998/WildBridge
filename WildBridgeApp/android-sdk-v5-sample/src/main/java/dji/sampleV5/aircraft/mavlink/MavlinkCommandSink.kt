package dji.sampleV5.aircraft.mavlink

/**
 * The commands both the HTTP surface and the MAVLink endpoint are allowed to execute — the single
 * command gate, so the two surfaces cannot drift in behaviour.
 *
 * Until now the MAVLink endpoint had no such route at all: it answered requests for messages and
 * refused everything else, which is what made it reviewable without a flight-safety argument. This
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

    /** Move the gimbal to the given rotation. Moves the gimbal, never the aircraft. */
    fun setGimbal(rotation: GimbalRotation): CommandResult

    /** Absolute zoom ratio. */
    fun setCameraZoom(zoomRatio: Float): CommandResult

    fun startVideoRecording(): CommandResult

    fun stopVideoRecording(): CommandResult

    /** Trip one shutter on the payload. Stores to the card; downloads nothing. */
    fun captureImage(): CommandResult

    /** Nudge the gimbal by a delta rather than to an absolute angle. */
    fun setGimbalRelative(pitchDeg: Double, yawDeg: Double): CommandResult

    /** Fire the laser rangefinder once. The reading surfaces on the telemetry stream. */
    fun measureLrf(): CommandResult

    /** Read the thermal spot temperature. No shutter, nothing stored. */
    fun captureTemperature(): CommandResult

    /** Release the payload. The one command here that moves something off the aircraft. */
    fun dropPayload(): CommandResult

    /**
     * Write one named setting.
     *
     * Named rather than typed because the settings surface is a string map on the HTTP side too;
     * the endpoint refuses names outside its allowlist before this is reached.
     */
    fun setParameter(name: String, value: Float): CommandResult
}

/**
 * A command that was accepted and is still running.
 *
 * [seq] is the id WildBridge's controller assigned to the movement, and is what distinguishes
 * this leg's arrival from the previous one's — the same guard the HTTP surface exposes, kept
 * because the underlying controller still works that way.
 */
internal data class PendingCommand(val kind: PendingKind, val seq: Long)

/** Which reach latch a [PendingCommand] is waiting on. */
internal enum class PendingKind { WAYPOINT, YAW, ALTITUDE }

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

/**
 * A command outcome plus the text a surface may render.
 *
 * [outcome] is the MAV_RESULT-shaped result the two surfaces share; [mavResult] is that same value
 * for callers that only need the wire value. [detail] is the human-readable text the HTTP surface
 * returns today, produced once by the command layer so the route table stops composing prose.
 * The MAVLink surface ignores [detail] and reads only [outcome].
 */
internal data class CommandResult(
    val outcome: MavlinkCommandOutcome,
    val detail: String? = null,
    /**
     * Set when the command has been accepted but has not finished yet.
     *
     * MAVLink's command protocol says a long-running command should be acknowledged with
     * MAV_RESULT_IN_PROGRESS and acknowledged again when it completes. That is a better fit than
     * WildBridge's reach latches, because the protocol correlates the completion to the request
     * itself: the stale-latch race the seq numbers exist to work around cannot arise when the
     * answer is addressed to the question.
     */
    val pending: PendingCommand? = null,
    /**
     * A scalar the command read back, carried in COMMAND_ACK's result_param2.
     *
     * Only meaningful for the read commands (rangefinder distance in centimetres, spot
     * temperature in hundredths of a degree); zero everywhere else. Integer because
     * result_param2 is an int32 — the scaling is what keeps a fractional reading intact.
     */
    val resultValue: Int = 0
) {
    val mavResult: Int get() = outcome.mavResult
}
