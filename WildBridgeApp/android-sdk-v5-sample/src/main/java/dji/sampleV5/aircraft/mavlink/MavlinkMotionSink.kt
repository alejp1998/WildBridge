package dji.sampleV5.aircraft.mavlink

/**
 * The flight-motion commands the MAVLink endpoint may execute.
 *
 * This is deliberately a separate surface from [MavlinkCommandSink]: the payload sink promises
 * "nothing here can move the aircraft", and keeping that true is what made the payload slice
 * reviewable without a flight-safety argument. Motion lives here, behind its own gate, so that
 * reviewing or disabling flight motion never touches the payload path.
 *
 * The gate is implemented by the host activity (where the prefs, authority latch and controllers
 * live) and must hold before any motion executes:
 *   1. the `wb_mav_0_allow_flight` preference is true (ships false);
 *   2. command authority is held (the Safety Computer has not seized control);
 *   3. the RC manual-override latch is clear, for the closed-loop commands that fight the pilot.
 */
internal interface MavlinkMotionSink {

    /** Begin a takeoff. */
    fun takeoff(): CommandResult

    /** Begin an auto-land at the current position. */
    fun land(): CommandResult

    /** Begin return-to-launch. */
    fun returnToHome(): CommandResult

    /** Fly to a position, holding the given heading on arrival. */
    fun reposition(
        latitudeDeg: Double,
        longitudeDeg: Double,
        altitudeMeters: Double,
        yawDeg: Double,
        groundSpeedMps: Double
    ): CommandResult

    /** Rotate in place to an absolute yaw, in degrees. */
    fun setYaw(yawDeg: Double): CommandResult

    /**
     * An arming request. DJI has no arm/disarm: motors spin up when a takeoff starts and stop
     * after touchdown. The request is acknowledged as a no-op (still behind the gate) so a ground
     * station's takeoff sequence does not abort on a refused arm.
     */
    fun arm(): CommandResult

    /** A disarming request, likewise acknowledged as a no-op behind the gate. */
    fun disarm(): CommandResult
}
