package dji.sampleV5.aircraft.mavlink

/**
 * WildBridge's own flight modes, reported in `HEARTBEAT.custom_mode`.
 *
 * These deliberately do **not** borrow another flight stack's mode numbers. RosettaDrone declares
 * itself an ArduPilot autopilot and reuses ArduCopter's `custom_mode` integers, which makes a
 * ground station render ArduCopter mode names for a DJI aircraft. WildBridge reports
 * `MAV_AUTOPILOT_INVALID` and owns its own list instead.
 *
 * The names below line up with the `MAV_STANDARD_MODE` enum so that the standard-modes protocol
 * (`AVAILABLE_MODES` / `CURRENT_MODE` / `MAV_CMD_DO_SET_STANDARD_MODE`) can advertise them by
 * their portable identity in a later phase. Until that lands a ground station will show the
 * number rather than a name — telemetry is read-only here, so nothing depends on it yet.
 *
 * @param guided the aircraft is being flown by something other than the pilot's sticks
 * @param manualInput the pilot on the sticks has authority
 */
internal enum class MavlinkFlightMode(
    val customMode: Int,
    val guided: Boolean,
    val manualInput: Boolean
) {
    UNKNOWN(0, guided = false, manualInput = false),
    POSITION_HOLD(1, guided = false, manualInput = true),
    ALTITUDE_HOLD(2, guided = false, manualInput = true),
    OFFBOARD(3, guided = true, manualInput = false),
    MISSION(4, guided = true, manualInput = false),
    TAKEOFF(5, guided = true, manualInput = false),
    LAND(6, guided = true, manualInput = false),
    SAFE_RECOVERY(7, guided = true, manualInput = false);

    companion object {
        /**
         * Map a DJI flight-mode name onto a WildBridge mode.
         *
         * When the manual-override latch is set the pilot has taken the sticks, so the reported
         * mode is a position-hold regardless of what the autonomous layer was doing — which is
         * the behaviour the mode model makes standard later, when the latch is replaced by this
         * transition outright.
         */
        fun fromDjiMode(djiMode: String, manualOverrideActive: Boolean): MavlinkFlightMode {
            if (manualOverrideActive) return POSITION_HOLD
            return when (djiMode.uppercase()) {
                "GPS", "GPS_ATTI", "P_GPS", "POSITION" -> POSITION_HOLD
                "ATTI", "ATTITUDE" -> ALTITUDE_HOLD
                "VIRTUAL_STICK", "VIRTUAL_STICK_CONTROL" -> OFFBOARD
                "WAYPOINT", "GPS_WAYPOINT", "AUTO" -> MISSION
                "TAKE_OFF", "TAKEOFF", "AUTO_TAKEOFF", "ASSISTED_TAKEOFF" -> TAKEOFF
                "AUTO_LANDING", "LANDING", "GO_DOWN" -> LAND
                "GO_HOME", "RETURN_TO_HOME", "RTH" -> SAFE_RECOVERY
                "MANUAL" -> POSITION_HOLD
                else -> UNKNOWN
            }
        }
    }
}
