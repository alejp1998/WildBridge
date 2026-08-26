package dji.sampleV5.aircraft.mavlink

/**
 * WildBridge's own flight modes, reported in `HEARTBEAT.custom_mode`.
 *
 * These deliberately do **not** borrow another flight stack's mode numbers. RosettaDrone declares
 * itself an ArduPilot autopilot and reuses ArduCopter's `custom_mode` integers, which makes a
 * ground station render ArduCopter mode names for a DJI aircraft. WildBridge reports
 * `MAV_AUTOPILOT_INVALID` and owns its own list instead.
 *
 * The names line up with `MAV_STANDARD_MODE` where a portable equivalent exists, so the
 * standard-modes protocol (`AVAILABLE_MODES` / `CURRENT_MODE` / `MAV_CMD_DO_SET_STANDARD_MODE`)
 * can advertise them by their portable identity in a later phase. [MANUAL] and [INTELLIGENT] have
 * no standard equivalent and stay custom-only rather than being forced into a nearby mode that
 * would misdescribe what the aircraft is doing.
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
    SAFE_RECOVERY(7, guided = true, manualInput = false),
    ORBIT(8, guided = true, manualInput = false),

    /** DJI's fully manual mode: no position or altitude assistance. */
    MANUAL(9, guided = false, manualInput = true),

    /** DJI intelligent-flight modes — follow, tap-fly, cinematic and friends. */
    INTELLIGENT(10, guided = true, manualInput = false);

    companion object {
        /**
         * Map a DJI flight-mode name onto a WildBridge mode.
         *
         * The names below are the actual `dji.sdk.keyvalue.value.flightcontroller.FlightMode`
         * constants from MSDK 5.18, read out of the SDK jar — an earlier version of this mapping
         * was written from plausible guesses and silently reported UNKNOWN against a real
         * aircraft, which reports `GPS_NORMAL` rather than `GPS`.
         *
         * When the manual-override latch is set the pilot has taken the sticks, so the reported
         * mode is a position hold regardless of what the autonomous layer was doing — which is
         * the behaviour the mode model makes standard later, when the latch is replaced by this
         * transition outright.
         */
        fun fromDjiMode(djiMode: String, manualOverrideActive: Boolean): MavlinkFlightMode {
            if (manualOverrideActive) return POSITION_HOLD
            return when (djiMode.uppercase()) {
                "GPS_NORMAL", "GPS_SPORT", "GPS_TRIPOD", "GPS_NOVICE",
                "APAS", "AUTO_AVOIDANCE" -> POSITION_HOLD

                "ATTI", "ATTI_LANDING" -> ALTITUDE_HOLD
                "MANUAL" -> MANUAL
                "VIRTUAL_STICK" -> OFFBOARD
                "WAYPOINT" -> MISSION
                "GO_HOME" -> SAFE_RECOVERY
                "AUTO_LANDING", "FORCE_LANDING" -> LAND
                "TAKE_OFF_READY", "AUTO_TAKE_OFF", "MOTOR_START" -> TAKEOFF
                "POI" -> ORBIT

                "SMART_FLIGHT", "SMART_FLY", "CLICK_GO", "FOLLOW_ME", "TAP_FLY",
                "QUICK_MOVIE", "MASTER_SHOT", "CINEMATIC", "DRAW", "PANO",
                "TIME_LAPSE" -> INTELLIGENT

                else -> UNKNOWN
            }
        }
    }
}
