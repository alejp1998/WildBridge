package dji.sampleV5.aircraft.mavlink

/**
 * One consistent read of the aircraft state, in plain units, with no DJI SDK types.
 *
 * The mavlink package stays free of SDK imports for the same reason `TelemetryCoordinator` does:
 * the mapping from DJI values to wire values is the part worth testing, and it should not need a
 * drone (or the SDK on the classpath) to exercise. The host activity builds this from the same
 * accessors that feed the JSON telemetry cache, so the two surfaces cannot report different
 * numbers for the same instant.
 *
 * Unknown values use the conventions the MAVLink field documentation specifies rather than a
 * plausible-looking zero — see the `INVALID_*` constants.
 */
internal data class MavlinkSnapshot(
    val droneName: String = "",

    // Position. Altitudes in metres: [altitudeAslM] above mean sea level, [altitudeAglM]
    // relative to the take-off point.
    val latitudeDeg: Double = 0.0,
    val longitudeDeg: Double = 0.0,
    val altitudeAslM: Double = 0.0,
    val altitudeAglM: Double = 0.0,

    // Velocity in the NED frame, metres per second. Down is positive.
    val velocityNorthMps: Double = 0.0,
    val velocityEastMps: Double = 0.0,
    val velocityDownMps: Double = 0.0,

    // Attitude and heading in degrees. [headingDeg] is true north, as DJI reports it.
    val rollDeg: Double = 0.0,
    val pitchDeg: Double = 0.0,
    val yawDeg: Double = 0.0,
    val headingDeg: Double = 0.0,

    val satelliteCount: Int = INVALID_SATELLITES,

    /** Battery charge 0..100, or [INVALID_BATTERY] when the SDK has not reported one yet. */
    val batteryPercent: Int = INVALID_BATTERY,
    /** Seconds of flight remaining, or 0 meaning "no estimate provided" per BATTERY_STATUS. */
    val remainingFlightTimeS: Int = 0,

    val homeLatitudeDeg: Double = 0.0,
    val homeLongitudeDeg: Double = 0.0,
    val homeAltitudeAslM: Double = 0.0,
    val homeSet: Boolean = false,

    /** DJI flight mode name, used only to derive the reported mode. */
    val flightMode: String = "UNKNOWN",

    /**
     * Motors running. DJI exposes no arm/disarm concept, so this is the only honest source for
     * the heartbeat's armed flag — deriving it from the flight mode instead reports armed on the
     * ground, which is what the ground station must never be told.
     */
    val motorsRunning: Boolean = false,

    val manualOverrideActive: Boolean = false,

    /**
     * Camera is recording. Reported as CAMERA_CAPTURE_STATUS so a ground station's record button
     * reflects what the aircraft is actually doing, including when recording was started from the
     * WildBridge UI or the RC rather than over MAVLink.
     */
    val isRecording: Boolean = false,

    // -- Payload and gimbal ------------------------------------------------------------------

    /** Gimbal attitude in the world frame, degrees. */
    val gimbalRollDeg: Double = 0.0,
    val gimbalPitchDeg: Double = 0.0,
    val gimbalYawDeg: Double = 0.0,

    /** Gimbal yaw relative to the aircraft body, degrees. DJI calls this the joint angle. */
    val gimbalJointYawDeg: Double = 0.0,

    /** Focal lengths in millimetres, or 0 when the payload does not report them. */
    val zoomFocalLengthMm: Int = 0,
    val opticalFocalLengthMm: Int = 0,
    val hybridFocalLengthMm: Int = 0,

    /** Last rangefinder reading in metres, or null when the laser has not locked. */
    val lrfDistanceM: Double? = null,
    val lrfTargetLatitudeDeg: Double? = null,
    val lrfTargetLongitudeDeg: Double? = null,
    val lrfTargetAltitudeM: Double? = null,

    // -- Authority and readiness -------------------------------------------------------------

    /** Pre-flight checks pass and a takeoff would be accepted. */
    val readyToTakeoff: Boolean = false,

    /** Why a takeoff would be refused, or "NONE". */
    val takeoffBlockReason: String = "",

    // -- DJI's smart-return budgets ----------------------------------------------------------

    val timeNeededToGoHomeS: Int = 0,
    val timeNeededToLandS: Int = 0,
    val totalFlightTimeS: Int = 0,
    val maxRadiusCanFlyAndGoHomeM: Double = 0.0,
    val batteryNeededToGoHomePercent: Int = 0,
    val batteryNeededToLandPercent: Int = 0
) {
    companion object {
        const val INVALID_BATTERY = -1
        const val INVALID_SATELLITES = -1

        /** GPS_RAW_INT / GLOBAL_POSITION_INT "unknown" for uint16 fields. */
        const val UINT16_UNKNOWN = 0xFFFF

        /** BATTERY_STATUS temperature "unknown" (INT16_MAX). */
        const val INT16_UNKNOWN = 32767
    }
}
