package dji.sampleV5.aircraft.mavlink

/**
 * Flies a stored mission. Implemented in the host activity, where the DJI SDK is.
 *
 * WildBridge has two ways to fly a path and they are genuinely different, so the executor is a
 * choice rather than an implementation detail:
 *
 *  - **Onboard** sequences the items itself through the PID waypoint controllers, one at a time,
 *    advancing when each is reached. It honours [MissionItem.noseForward] per item, which the DJI
 *    path cannot, and it is the default.
 *  - **DJI native** hands the whole list to the aircraft's own wayline engine. Smoother, and it
 *    keeps flying if the phone's attention lapses, but the heading behaviour is DJI's.
 *
 * This interface can move the aircraft. Implementations must pass the same gates as
 * [MavlinkMotionSink] — the allow-flight preference, control authority, and the RC override latch.
 */
internal interface MavlinkMissionSink {

    /**
     * Whether a plan is being flown right now.
     *
     * Read before any command that takes the aircraft elsewhere, so a guided command can stop the
     * sequencer rather than racing it.
     */
    val isRunning: Boolean


    /** Begin flying [items] from index [startIndex]. */
    fun startMission(items: List<MissionItem>, startIndex: Int, executor: MissionExecutor): CommandResult

    /** Stop flying and hold. Does not clear the stored plan. */
    fun stopMission(): CommandResult

    /** Called by the endpoint so the sink can report progress back as it flies. */
    fun setProgressListener(listener: MissionProgressListener?)
}

/** Which of WildBridge's two path-following implementations flies the plan. */
internal enum class MissionExecutor(val prefValue: String) {
    /** WildBridge sequences items through the PID waypoint controllers. */
    ONBOARD("onboard"),

    /** DJI's own wayline engine flies the whole list. */
    DJI_NATIVE("dji_native");

    companion object {
        /**
         * Onboard by default. It is the executor that honours per-item heading, reports an exact
         * MISSION_ITEM_REACHED, and keeps mission state where MAVLink expects it — on the vehicle.
         */
        fun fromPref(value: String?): MissionExecutor =
            entries.firstOrNull { it.prefValue == value } ?: ONBOARD
    }
}

/** How the sink tells the endpoint where the mission has got to. */
internal interface MissionProgressListener {
    /** The executor has started flying item [seq]. */
    fun onItemStarted(seq: Int)

    /** Item [seq] has been reached. */
    fun onItemReached(seq: Int)

    /** The plan finished, or stopped early. */
    fun onMissionFinished(completed: Boolean)
}
