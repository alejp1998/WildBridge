package dji.sampleV5.aircraft.mavlink

/**
 * Configuration for one MAVLink endpoint, shaped after PX4's MAVLink instance parameters.
 *
 * PX4 does not have "a MAVLink connection" — it has numbered instances, each switched entirely by
 * parameter: `MAV_n_CONFIG` chooses the port or disables the instance, `MAV_n_MODE` chooses a
 * stream profile, `MAV_n_RATE` caps bandwidth. WildBridge copies that shape so the endpoint can be
 * enabled per aircraft rather than per build, and so a second instance can later serve the Safety
 * Computer with its own profile and rate.
 *
 * [enabled] defaults to false: a second control surface on a flying platform ships dark and is
 * switched on deliberately.
 */
internal data class MavlinkEndpointConfig(
    val enabled: Boolean = false,
    /** Where to send. Empty means broadcast on the local subnet. */
    val targetHost: String = "",
    val targetPort: Int = DEFAULT_GCS_PORT,
    /** Local port to bind, so the endpoint also learns peers that talk to it first. */
    val listenPort: Int = DEFAULT_GCS_PORT,
    val mode: Profile = Profile.NORMAL,
    /** MAVLink system id. One per aircraft, the way a GCS distinguishes vehicles. */
    val systemId: Int = DEFAULT_SYSTEM_ID
) {
    /**
     * Stream profile, the equivalent of `MAV_n_MODE`. Each profile picks a different set of
     * messages and rates; [MINIMAL] exists for links where bandwidth matters more than smoothness.
     */
    enum class Profile(val prefValue: String) {
        NORMAL("normal"),
        MINIMAL("minimal");

        companion object {
            fun fromPref(value: String?): Profile =
                entries.firstOrNull { it.prefValue == value } ?: NORMAL
        }
    }

    companion object {
        /** The port QGroundControl listens on by default. */
        const val DEFAULT_GCS_PORT = 14550
        const val DEFAULT_SYSTEM_ID = 1

        const val PREF_ENABLED = "wb_mav_0_enabled"
        const val PREF_HOST = "wb_mav_0_host"
        const val PREF_PORT = "wb_mav_0_port"
        const val PREF_MODE = "wb_mav_0_mode"
        const val PREF_SYSTEM_ID = "wb_mav_0_sysid"
    }
}
