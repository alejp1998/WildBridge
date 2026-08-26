package dji.sampleV5.aircraft.mavlink

import android.util.Log
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import java.util.concurrent.ConcurrentHashMap
import kotlin.concurrent.thread

/**
 * MAVLink 2 telemetry endpoint: streams the phase-1 message set over UDP so a stock ground station
 * shows the aircraft with no plugin and no configuration file.
 *
 * Inbound traffic is handled, but only just: a datagram can teach the endpoint where a ground
 * station lives, ask for the camera and video-stream descriptions, and — through the host's
 * gated motion sink — request takeoff, land, return-to-launch, reposition and yaw. Everything
 * else is acknowledged `MAV_RESULT_UNSUPPORTED` and dropped. Motion reaches the aircraft only
 * via [MavlinkMotionSink], which is null (and therefore refuses) unless the host enables it
 * behind `wb_mav_0_allow_flight`. See [handleCommand] and [executeCommand].
 *
 * Messages are streamed at per-message rates rather than one uniform tick. A single rate is what
 * makes a link feel sluggish: attitude needs to be smooth, battery does not, and sending both at
 * the same interval gets the worst of each.
 */
internal class MavlinkTelemetryEndpoint(
    private val config: MavlinkEndpointConfig,
    private val snapshotProvider: () -> MavlinkSnapshot,
    /**
     * The RTSP URL a ground station should play, or null when no stream is running. Advertised as
     * VIDEO_STREAM_INFORMATION so QGroundControl configures its own video source instead of the
     * operator typing a URL.
     */
    private val videoStreamProvider: () -> MavlinkVideoStream? = { null },
    /**
     * Read-only parameters to publish, typically the active control profile. Empty is valid but
     * costs a ground station its initial connect — see [handleListRequest].
     */
    private val parameterProvider: () -> List<Pair<String, Float>> = { emptyList() },
    /**
     * Executes the payload and camera commands. Null means the endpoint stays request-only, which
     * is how it behaved before commands existed and remains a valid configuration.
     */
    private val commandSink: MavlinkCommandSink? = null,
    /**
     * Executes flight-motion commands, gated by the host. Null means motion is refused — the safe
     * default until a flying platform deliberately enables it behind `wb_mav_0_allow_flight`.
     */
    private val motionSink: MavlinkMotionSink? = null,
    /** Flies stored plans. Null leaves missions uploadable but not startable. */
    private val missionSink: MavlinkMissionSink? = null
) {
    private val framer = MavlinkFramer(config.systemId)

    /**
     * The camera is a separate MAVLink component, which is not decoration: QGroundControl only
     * looks for cameras on component ids 100..105, so a stream advertised from the autopilot
     * component is never discovered.
     */
    private val cameraFramer = MavlinkFramer(config.systemId, Mav.COMP_ID_CAMERA)
    private val bootNanos = System.nanoTime()

    /** The uploaded plan and the upload handshake's state. */
    private val missions = MavlinkMissionStore()

    /** Who to answer during an upload or download; set from whoever started it. */
    @Volatile
    private var missionPeerSystem = 0

    @Volatile
    private var missionPeerComponent = 0

    private var socket: DatagramSocket? = null
    private var senderThread: Thread? = null
    private var receiverThread: Thread? = null

    @Volatile
    private var running = false

    /** Explicitly configured destination, if any. */
    @Volatile
    private var configuredTarget: InetSocketAddress? = null

    /**
     * Addresses of peers that have contacted us, so responses reach every interested ground
     * station rather than only the last one that spoke.
     */
    private val discoveredTargets: MutableSet<InetSocketAddress> = ConcurrentHashMap.newKeySet()

    /** Set once a peer has been seen, so the UI can show whether a GCS is actually attached. */
    @Volatile
    var peerAddress: String? = null
        private set

    /** Fires the first time a ground station is heard from. */
    var onPeerDiscovered: ((String) -> Unit)? = null

    /** Photos taken since boot, reported as CAMERA_CAPTURE_STATUS.image_count. */
    private val imageCount = java.util.concurrent.atomic.AtomicInteger(0)

    /** True while a shutter is in flight, so capture status reports it honestly. */
    @Volatile
    private var capturing = false

    /**
     * Announce the result of a shutter that was started earlier.
     *
     * Capture is asynchronous by necessity — tripping a shutter and waiting for the file to appear
     * takes seconds, and the endpoint's receive thread cannot block for that long. The command is
     * acknowledged immediately and the outcome arrives here, which is exactly the split
     * CAMERA_IMAGE_CAPTURED exists for: `capture_result` reports whether the photo happened.
     */
    fun reportImageCaptured(success: Boolean, fileName: String) {
        capturing = false
        val index = if (success) imageCount.incrementAndGet() else imageCount.get()
        val snapshot = runCatching { snapshotProvider() }.getOrDefault(MavlinkSnapshot())
        sendOnce(
            MavlinkMsgId.CAMERA_IMAGE_CAPTURED,
            MavlinkMessages.cameraImageCaptured(
                snapshot, timeBootMs(), index, success, fileName
            ),
            fromCamera = true
        )
        Log.i(TAG, "Image captured: success=$success file=$fileName index=$index")
    }

    /** Called when a shutter is started, so capture status shows it in progress. */
    fun reportCaptureStarted() {
        capturing = true
    }

    fun start() {
        if (running) return
        running = true

        senderThread = thread(name = "MavlinkEndpoint-tx", start = true) {
            runCatching { openSocketAndStream() }
                .onFailure { error -> Log.e(TAG, "Endpoint stopped: ${error.message}", error) }
        }
    }

    fun stop() {
        running = false
        onPeerDiscovered = null
        runCatching {
            socket?.close()
            socket = null
            if (Thread.currentThread() != receiverThread) receiverThread?.join(JOIN_TIMEOUT_MS)
            if (Thread.currentThread() != senderThread) senderThread?.join(JOIN_TIMEOUT_MS)
        }.onFailure { error -> Log.w(TAG, "Error stopping endpoint: ${error.message}") }
        receiverThread = null
        senderThread = null
    }

    private fun openSocketAndStream() {
        val bound = DatagramSocket(config.listenPort).apply { broadcast = true }
        socket = bound
        configuredTarget = resolveConfiguredTarget()
        Log.i(
            TAG,
            "MAVLink 2 endpoint up: sysid=${config.systemId} listen=${config.listenPort} " +
                "target=${configuredTarget?.toString() ?: "broadcast"} profile=${config.mode.prefValue}"
        )

        receiverThread = thread(name = "MavlinkEndpoint-rx", start = true) { receiveLoop(bound) }

        sendOnce(MavlinkMsgId.STATUSTEXT, MavlinkMessages.statusText(Mav.SEVERITY_INFO, bootBanner()))
        streamLoop()
    }

    private fun bootBanner(): String {
        val name = snapshotProvider().droneName.ifBlank { "WildBridge" }
        return "$name online (WildBridge, MAVLink 2)"
    }

    private fun resolveConfiguredTarget(): InetSocketAddress? {
        if (config.targetHost.isBlank()) return null
        return runCatching {
            InetSocketAddress(InetAddress.getByName(config.targetHost), config.targetPort)
        }.onFailure { error ->
            Log.w(TAG, "Cannot resolve ${config.targetHost}: ${error.message}")
        }.getOrNull()
    }

    /**
     * Reads inbound datagrams purely to learn where a ground station lives. QGroundControl creates
     * a UDP link as soon as it receives traffic, so this mostly matters for the reverse case: a
     * GCS that heartbeats at us before we know its address.
     */
    private fun receiveLoop(bound: DatagramSocket) {
        val buffer = ByteArray(RECEIVE_BUFFER_BYTES)
        while (running && !bound.isClosed) {
            val packet = DatagramPacket(buffer, buffer.size)
            val received = runCatching { bound.receive(packet); true }
                .getOrElse { error ->
                    if (running && error !is SocketTimeoutException && bound.isClosed.not()) {
                        Log.d(TAG, "Receive ended: ${error.message}")
                    }
                    false
                }
            if (!received) break
            if (packet.length > 0 && buffer[0] == MavlinkFramer.MAGIC_V2) {
                notePeer(InetSocketAddress(packet.address, packet.port))
                MavlinkInbound.parseCommand(buffer, packet.length)?.let(::handleCommand)
                MavlinkInbound.parseSetMode(buffer, packet.length)?.let(::handleSetMode)
                MavlinkInbound.parseListRequest(buffer, packet.length)?.let(::handleListRequest)
                MavlinkInbound.parseManualControl(buffer, packet.length)?.let(::handleManualControl)
                MavlinkInbound.parseParamSet(buffer, packet.length)?.let(::handleParamSet)
                handleMissionFrame(buffer, packet.length)
            }
        }
    }

    private fun notePeer(address: InetSocketAddress) {
        if (!discoveredTargets.add(address)) return
        val label = "${address.address.hostAddress}:${address.port}"
        peerAddress = label
        Log.i(TAG, "Ground station discovered at $label")
        onPeerDiscovered?.invoke(label)
    }

    private fun streamLoop() {
        val streams = buildStreams()
        var lastTick = System.nanoTime()
        // Last reported aircraft state, so transitions log once instead of every tick.
        var lastArmed = false
        var lastLanded = Mav.LANDED_STATE_ON_GROUND

        while (running) {
            val now = System.nanoTime()
            val elapsedMs = (now - lastTick) / NANOS_PER_MILLI
            lastTick = now

            val snapshot = runCatching { snapshotProvider() }.getOrElse {
                Log.w(TAG, "Snapshot failed: ${it.message}")
                MavlinkSnapshot()
            }

            // Field diagnostic: log armed and landed-state transitions. QGC derives its "Flying"
            // indicator and the Land/RTL button enablement from EXTENDED_SYS_STATE, so if a ground
            // station ever shows the wrong state these two lines show exactly what was sent.
            if (snapshot.motorsRunning != lastArmed) {
                lastArmed = snapshot.motorsRunning
                Log.i(TAG, "State change: motors/armed=$lastArmed")
            }
            val mode = MavlinkFlightMode.fromDjiMode(snapshot.flightMode, snapshot.manualOverrideActive)
            val landed = when {
                !snapshot.motorsRunning -> Mav.LANDED_STATE_ON_GROUND
                mode == MavlinkFlightMode.LAND -> Mav.LANDED_STATE_LANDING
                mode == MavlinkFlightMode.TAKEOFF -> Mav.LANDED_STATE_TAKEOFF
                else -> Mav.LANDED_STATE_IN_AIR
            }
            if (landed != lastLanded) {
                lastLanded = landed
                Log.i(TAG, "State change: landed_state=$landed mode=${mode.name}")
            }

            for (stream in streams) {
                if (stream.due(elapsedMs) && stream.shouldSend(snapshot)) {
                    runCatching { sendOnce(stream.messageId, stream.build(snapshot), stream.camera) }
                        .onFailure { error -> Log.w(TAG, "Send failed: ${error.message}") }
                }
            }

            runCatching { Thread.sleep(TICK_MS) }.onFailure {
                Thread.currentThread().interrupt()
                running = false
            }
        }
    }

    /**
     * Per-message rates. The values follow what PX4 streams on a normal GCS link: attitude fast
     * enough to look smooth, position a little slower, status and battery at 1 Hz.
     */
    private fun buildStreams(): List<Stream> {
        val fast = config.mode != MavlinkEndpointConfig.Profile.MINIMAL
        return listOf(
            Stream(MavlinkMsgId.HEARTBEAT, HEARTBEAT_INTERVAL_MS) { MavlinkMessages.heartbeat(it) },
            Stream(MavlinkMsgId.SYS_STATUS, SLOW_INTERVAL_MS) { MavlinkMessages.sysStatus(it) },
            // landed_state is QGC's source of the flying state; without it the Fly View never
            // offers Land/RTL because the vehicle is never "flying".
            Stream(MavlinkMsgId.EXTENDED_SYS_STATE, SLOW_INTERVAL_MS) {
                MavlinkMessages.extendedSysState(it)
            },
            Stream(MavlinkMsgId.BATTERY_STATUS, SLOW_INTERVAL_MS) { MavlinkMessages.batteryStatus(it) },
            // Only once DJI actually has a home point. Before then the SDK reports an
            // uninitialised location, and a HOME_POSITION carrying it would put a home marker at a
            // fictional place on the ground station's map — worse than showing no home at all.
            Stream(MavlinkMsgId.HOME_POSITION, HOME_INTERVAL_MS, sendIf = { it.homeSet }) {
                MavlinkMessages.homePosition(it)
            },
            Stream(MavlinkMsgId.GPS_RAW_INT, POSITION_INTERVAL_MS) {
                MavlinkMessages.gpsRawInt(it, unixTimeUsec())
            },
            Stream(MavlinkMsgId.GLOBAL_POSITION_INT, POSITION_INTERVAL_MS) {
                MavlinkMessages.globalPositionInt(it, timeBootMs())
            },
            Stream(MavlinkMsgId.VFR_HUD, POSITION_INTERVAL_MS) { MavlinkMessages.vfrHud(it) },
            Stream(
                MavlinkMsgId.ATTITUDE,
                if (fast) ATTITUDE_INTERVAL_MS else POSITION_INTERVAL_MS
            ) { MavlinkMessages.attitude(it, timeBootMs()) },
            Stream(MavlinkMsgId.AUTOPILOT_VERSION, VERSION_INTERVAL_MS) {
                MavlinkMessages.autopilotVersion()
            },
            // Streamed as well as served on request: recording can be started from the WildBridge
            // UI or the RC, and a ground station that only polled after its own commands would
            // never notice.
            Stream(MavlinkMsgId.CAMERA_CAPTURE_STATUS, CAPTURE_STATUS_INTERVAL_MS, camera = true) {
                MavlinkMessages.cameraCaptureStatus(
                    timeBootMs(), it.isRecording, capturing, imageCount.get()
                )
            },
            // Streamed as well as served on request, for the same reason as CAMERA_CAPTURE_STATUS:
            // the WHIP publish starts asynchronously (when a telemetry client attaches), so a
            // ground station that asked before the stream was up gets DENIED and, without this,
            // never learns the stream exists — which is why switching the active vehicle left the
            // video stuck on the first drone.
            Stream(
                MavlinkMsgId.VIDEO_STREAM_INFORMATION,
                SLOW_INTERVAL_MS,
                camera = true,
                sendIf = { videoStreamProvider()?.uri?.isNotBlank() == true }
            ) {
                val stream = videoStreamProvider()!!
                MavlinkMessages.videoStreamInformation(
                    uri = stream.uri,
                    name = stream.name,
                    framerate = stream.framerate,
                    widthPx = stream.widthPx,
                    heightPx = stream.heightPx
                )
            },
            // Only once a plan exists: a ground station with no plan does not need telling
            // twice a second that there is still no plan.
            Stream(
                MavlinkMsgId.MISSION_CURRENT,
                MISSION_CURRENT_INTERVAL_MS,
                sendIf = { missions.count() > 0 }
            ) {
                MavlinkMessages.missionCurrent(
                    seq = missions.currentIndex(),
                    total = missions.count(),
                    state = missions.missionState(),
                    planId = missions.currentPlanId()
                )
            },
            Stream(MavlinkMsgId.CURRENT_MODE, CURRENT_MODE_INTERVAL_MS) {
                MavlinkMessages.currentMode(
                    MavlinkFlightMode.fromDjiMode(it.flightMode, it.manualOverrideActive)
                )
            },
            // The camera component's own heartbeat. Without it QGroundControl never asks for
            // CAMERA_INFORMATION, and the video stream is never discovered.
            Stream(MavlinkMsgId.HEARTBEAT, HEARTBEAT_INTERVAL_MS, camera = true) {
                MavlinkMessages.cameraHeartbeat()
            }
        )
    }

    /**
     * Answer the only inbound commands this endpoint implements: "send me that message".
     *
     * **This is the whole command surface, and it is an allowlist by construction.** Every command
     * outside the four below is acknowledged with `MAV_RESULT_UNSUPPORTED` and nothing else
     * happens — a takeoff, a mode change or a waypoint reaches no code path here, because none
     * exists. That property is what keeps this phase reviewable without a flight-safety argument,
     * and it should be preserved deliberately rather than eroded when commands do arrive: the
     * intent is that the allowlist grows by decision, not by a `when` branch quietly falling
     * through.
     *
     * QGroundControl asks for camera information two ways, alternating between
     * `MAV_CMD_REQUEST_MESSAGE` and the deprecated `MAV_CMD_REQUEST_CAMERA_INFORMATION`, so both
     * are handled or discovery stalls on the retry loop.
     */
    private fun handleCommand(command: MavlinkCommand) {
        val forCamera = command.targetComponent == Mav.COMP_ID_CAMERA ||
            command.targetComponent == 0

        val result: CommandResult = when (command.command) {
            Mav.CMD_REQUEST_MESSAGE -> asResult(sendRequestedMessage(
                command.param1.toInt(), forCamera, command.param2.toInt()
            ))
            Mav.CMD_REQUEST_CAMERA_INFORMATION ->
                asResult(if (forCamera) sendCameraInformation() else Mav.RESULT_UNSUPPORTED)
            Mav.CMD_REQUEST_VIDEO_STREAM_INFORMATION ->
                asResult(if (forCamera) sendVideoStreamInformation() else Mav.RESULT_UNSUPPORTED)
            Mav.CMD_REQUEST_CAMERA_SETTINGS ->
                asResult(if (forCamera) sendCameraSettings() else Mav.RESULT_UNSUPPORTED)
            Mav.CMD_REQUEST_STORAGE_INFORMATION ->
                asResult(if (forCamera) sendStorageInformation() else Mav.RESULT_UNSUPPORTED)
            Mav.CMD_REQUEST_VIDEO_STREAM_STATUS ->
                asResult(if (forCamera) sendVideoStreamStatus() else Mav.RESULT_UNSUPPORTED)
            Mav.CMD_REQUEST_CAMERA_CAPTURE_STATUS ->
                asResult(if (forCamera) sendCameraCaptureStatus() else Mav.RESULT_UNSUPPORTED)
            else -> executeCommand(command)
        }

        val ack = MavlinkMessages.commandAck(
            command = command.command,
            result = result.mavResult,
            targetSystem = command.senderSystem,
            targetComponent = command.senderComponent,
            resultValue = result.resultValue
        )
        // The ack must come from the component that was addressed, or the requester will not
        // match it to its request.
        sendOnce(MavlinkMsgId.COMMAND_ACK, ack, fromCamera = forCamera)
    }

    /**
     * Execute one command, or refuse it.
     *
     * Payload/camera commands go through [MavlinkCommandSink]; flight-motion commands go through
     * [MavlinkMotionSink], which is null (and therefore refuses motion) unless the host enables it
     * behind `wb_mav_0_allow_flight`. A command not named here gets `MAV_RESULT_UNSUPPORTED`.
     */
    /** Wrap a bare MAV_RESULT from the message-request paths, which read nothing back. */
    private fun asResult(mavResult: Int): CommandResult = CommandResult(
        when (mavResult) {
            Mav.RESULT_ACCEPTED -> MavlinkCommandOutcome.ACCEPTED
            Mav.RESULT_DENIED -> MavlinkCommandOutcome.DENIED
            Mav.RESULT_UNSUPPORTED -> MavlinkCommandOutcome.UNSUPPORTED
            else -> MavlinkCommandOutcome.FAILED
        }
    )

    private fun executeCommand(command: MavlinkCommand): CommandResult {
        val sink = commandSink
        if (sink == null) {
            Log.d(TAG, "Refusing command ${command.command}: no command sink configured")
            return CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
        }
        val unsupported = CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
        val result = runCatching {
            when (command.command) {
                // param1 pitch, param2 yaw, both degrees.
                Mav.CMD_DO_GIMBAL_MANAGER_PITCHYAW ->
                    sink.setGimbal(
                        GimbalRotation(
                            mode = GimbalRotationMode.ABSOLUTE,
                            pitchDeg = command.param1.toDouble(),
                            rollDeg = 0.0,
                            yawDeg = command.param2.toDouble(),
                            // DO_GIMBAL_MANAGER_PITCHYAW sets pitch and yaw; roll is not part of
                            // the command. An axis the gimbal does not have is a no-op on the
                            // aircraft, so there is no capability check to make.
                            pitchIgnored = false,
                            rollIgnored = true,
                            yawIgnored = false
                        )
                    )

                // param2 is the zoom value for MAV_ZOOM_TYPE_RANGE / _CONTINUOUS.
                Mav.CMD_SET_CAMERA_ZOOM -> sink.setCameraZoom(command.param2)

                Mav.CMD_VIDEO_START_CAPTURE -> sink.startVideoRecording()
                Mav.CMD_VIDEO_STOP_CAPTURE -> sink.stopVideoRecording()
                Mav.CMD_IMAGE_START_CAPTURE -> sink.captureImage()

                // Flight motion, executed through the host's gated MavlinkMotionSink. A null sink
                // (motion disabled) is refused, and the gate itself returns DENIED when
                // wb_mav_0_allow_flight is off or the Safety Computer holds authority.
                Mav.CMD_MISSION_START -> startStoredMission(command.param1.toInt())

                Mav.CMD_DO_SET_MISSION_CURRENT -> {
                    missions.setCurrent(command.param1.toInt())
                    CommandResult(MavlinkCommandOutcome.ACCEPTED)
                }

                Mav.CMD_NAV_TAKEOFF -> motionSink?.takeoff(
                    // param7 is the requested altitude; NaN or non-positive means "use the
                    // aircraft's default", which is what a bare takeoff does.
                    command.param7.takeIf { it.isFinite() && it > 0f }
                ) ?: unsupported
                Mav.CMD_NAV_LAND -> motionSink?.land() ?: unsupported
                Mav.CMD_NAV_RETURN_TO_LAUNCH -> motionSink?.returnToHome() ?: unsupported
                Mav.CMD_DO_REPOSITION -> motionSink?.reposition(
                    // Not param5/param6: those are floats, and a goto deserves the full
                    // precision a COMMAND_INT actually carried.
                    latitudeDeg = command.latitudeDeg,
                    longitudeDeg = command.longitudeDeg,
                    altitudeMeters = command.param7.toDouble(),
                    yawDeg = command.param4.toDouble(),
                    groundSpeedMps = command.param1.toDouble()
                ) ?: unsupported
                Mav.CMD_CONDITION_YAW -> motionSink?.setYaw(command.param1.toDouble()) ?: unsupported

                // param7 is the target altitude; param1 (rate) is DJI's to choose.
                Mav.CMD_CONDITION_CHANGE_ALT ->
                    motionSink?.setAltitude(command.param7.toDouble()) ?: unsupported

                Mav.CMD_DO_GRIPPER ->
                    if (command.param2.toInt() == Mav.GRIPPER_ACTION_RELEASE) {
                        sink.dropPayload()
                    } else {
                        // Only release is possible: the drop port lets go, it cannot take hold.
                        CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
                    }

                Mav.CMD_USER_1 -> when (command.param1) {
                    Mav.USER1_GIMBAL_RELATIVE ->
                        sink.setGimbalRelative(command.param2.toDouble(), command.param3.toDouble())
                    Mav.USER1_RELEASE_MANUAL_OVERRIDE ->
                        motionSink?.releaseManualOverride() ?: unsupported
                    else -> CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
                }

                Mav.CMD_USER_2 -> when (command.param1) {
                    Mav.USER2_LRF_MEASURE -> sink.measureLrf()
                    Mav.USER2_CAPTURE_TEMPERATURE -> sink.captureTemperature()
                    else -> CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
                }

                // DJI has no arm/disarm: the aircraft arms when a takeoff actually starts. QGC's
                // PX4 plugin arms right after NAV_TAKEOFF is accepted, so this is acknowledged
                // (still behind the gate) rather than refused, or takeoff aborts on the error.
                Mav.CMD_COMPONENT_ARM_DISARM ->
                    if (command.param1 >= 0.5f) motionSink?.arm() ?: unsupported
                    else motionSink?.disarm() ?: unsupported

                // QGC's APM plugin (and some PX4 flows) request modes this way; QGC's PX4 plugin
                // normally sends SET_MODE instead — both land in the same mapper. The packed mode
                // numbers fit a float exactly (low 16 bits are zero), so the round trip is lossless.
                Mav.CMD_DO_SET_MODE -> modeResult(command.param2.toInt())

                else -> {
                    Log.d(TAG, "Refusing unsupported command ${command.command}")
                    CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
                }
            }
        }.getOrElse { error ->
            Log.w(TAG, "Command ${command.command} failed: ${error.message}", error)
            CommandResult(MavlinkCommandOutcome.FAILED)
        }
        if (result.outcome != MavlinkCommandOutcome.UNSUPPORTED) {
            Log.i(TAG, "Command ${command.command} -> ${result.outcome}")
        }
        return result
    }

    /**
     * A SET_MODE request from a ground station. QGC's PX4 firmware plugin sends this (not
     * DO_SET_MODE) when its Land / RTL buttons are pressed, and waits for the heartbeat to show
     * the requested mode — which it does, because acting on the request makes DJI report the
     * matching mode.
     */
    private fun handleSetMode(setMode: MavlinkSetMode) {
        if (setMode.targetSystem != 0 && setMode.targetSystem != config.systemId) return
        modeResult(setMode.customMode)
    }

    /**
     * Turn a requested PX4 mode number into action. Only the modes with a real DJI equivalent are
     * executed; the rest are refused, since DJI offers no way to change its flight mode remotely.
     */
    private fun modeResult(customMode: Int): CommandResult {
        val requested = MavlinkFlightMode.fromPx4Mode(customMode)
        if (requested == null) {
            Log.d(TAG, "Refusing mode request 0x${customMode.toString(16)}: not a WildBridge mode")
            return CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
        }
        val motion = motionSink
        if (motion == null) {
            Log.d(TAG, "Refusing mode request ${requested.displayName}: motion disabled")
            return CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
        }
        val result = when (requested) {
            // A mode request carries no height, unlike MAV_CMD_NAV_TAKEOFF's param7.
            MavlinkFlightMode.TAKEOFF -> motion.takeoff(altitudeM = null)
            MavlinkFlightMode.LAND -> motion.land()
            MavlinkFlightMode.SAFE_RECOVERY -> motion.returnToHome()

            // An abort is a mode change to position hold on this wire; see abortToPositionHold.
            MavlinkFlightMode.POSITION_HOLD -> motion.abortToPositionHold()

            // Offboard is what DJI calls virtual stick: the mode that accepts MANUAL_CONTROL.
            MavlinkFlightMode.OFFBOARD -> motion.enableOffboard()
            else -> {
                Log.d(TAG, "Refusing mode request ${requested.displayName}: no DJI equivalent")
                CommandResult(MavlinkCommandOutcome.UNSUPPORTED)
            }
        }
        if (result.outcome != MavlinkCommandOutcome.UNSUPPORTED) {
            Log.i(TAG, "Mode request ${requested.displayName} -> ${result.outcome}")
        }
        return result
    }

    private fun missionExecutor(): MissionExecutor = config.missionExecutor

    /**
     * Begin flying the stored plan.
     *
     * Refused rather than silently doing nothing when there is no plan or no executor: a ground
     * station that pressed start deserves to know why it did not move.
     */
    private fun startStoredMission(startIndex: Int): CommandResult {
        val sink = missionSink
            ?: return CommandResult(MavlinkCommandOutcome.UNSUPPORTED, "No mission executor")
        val items = missions.snapshot()
        if (items.isEmpty()) {
            return CommandResult(MavlinkCommandOutcome.DENIED, "No mission stored")
        }
        val from = startIndex.coerceIn(0, items.size - 1)
        sink.setProgressListener(missionProgress)
        val result = sink.startMission(items, from, missionExecutor())
        if (result.outcome == MavlinkCommandOutcome.ACCEPTED) {
            missions.setCurrent(from)
            Log.i(TAG, "Mission started at item $from of ${items.size}")
        }
        return result
    }

    /**
     * Drive the mission upload/download conversation.
     *
     * The protocol is a dialogue rather than a single message, and both ends can start one, so
     * this is written as a set of frame handlers over the shared [missions] state rather than as
     * a linear flow. Every reply goes to whoever spoke, not to a remembered peer, so two ground
     * stations cannot end up answering each other's requests.
     */
    private fun handleMissionFrame(data: ByteArray, length: Int) {
        MavlinkInbound.parseMissionCount(data, length)?.let { return handleMissionCount(it) }
        MavlinkInbound.parseMissionItem(data, length)?.let { return handleMissionItem(it) }
        if (MavlinkInbound.isMissionClearAll(data, length)) {
            missions.clear()
            missionSink?.stopMission()
            sendOnce(
                MavlinkMsgId.MISSION_ACK,
                MavlinkMessages.missionAck(
                    MissionResult.ACCEPTED, missionPeerSystem, missionPeerComponent
                )
            )
            Log.i(TAG, "Mission cleared")
        }
    }

    private fun handleMissionCount(count: MavlinkMissionCount) {
        missionPeerSystem = count.senderSystem
        missionPeerComponent = count.senderComponent
        val refusal = missions.beginUpload(count.count, count.missionType)
        if (refusal != null) {
            Log.i(TAG, "Refusing mission upload of ${count.count}: result $refusal")
            sendMissionAck(refusal)
            return
        }
        Log.i(TAG, "Mission upload started: ${count.count} items")
        if (count.count == 0) {
            // Nothing to request; a zero-item upload is how a plan is cleared.
            sendMissionAck(MissionResult.ACCEPTED)
            return
        }
        requestNextMissionItem()
    }

    private fun handleMissionItem(uploaded: MavlinkMissionItem) {
        missionPeerSystem = uploaded.senderSystem
        missionPeerComponent = uploaded.senderComponent
        val refusal = missions.acceptItem(uploaded.item)
        if (refusal != null) {
            // Naming the reason matters: a ground station can show which item it must change,
            // rather than the upload appearing to vanish.
            Log.i(TAG, "Rejecting item ${uploaded.item.seq} (cmd ${uploaded.item.command}): $refusal")
            missions.abortUpload()
            sendMissionAck(refusal)
            return
        }
        if (missions.uploadComplete()) {
            missions.commitUpload()
            Log.i(TAG, "Mission upload complete: ${missions.count()} items stored")
            sendMissionAck(MissionResult.ACCEPTED)
        } else {
            requestNextMissionItem()
        }
    }

    private fun requestNextMissionItem() {
        val next = missions.nextRequestIndex() ?: return
        sendOnce(
            MavlinkMsgId.MISSION_REQUEST_INT,
            MavlinkMessages.missionRequestInt(next, missionPeerSystem, missionPeerComponent)
        )
    }

    private fun sendMissionAck(result: Int) {
        sendOnce(
            MavlinkMsgId.MISSION_ACK,
            MavlinkMessages.missionAck(result, missionPeerSystem, missionPeerComponent)
        )
    }

    /** Send the stored plan back, item by item, in answer to a MISSION_REQUEST_LIST. */
    private fun sendMissionList(request: MavlinkListRequest) {
        missionPeerSystem = request.senderSystem
        missionPeerComponent = request.senderComponent
        val items = missions.snapshot()
        sendOnce(
            MavlinkMsgId.MISSION_COUNT,
            MavlinkMessages.missionCount(
                count = items.size,
                targetSystem = request.senderSystem,
                targetComponent = request.senderComponent,
                missionType = request.missionType
            )
        )
        // The stored items are returned verbatim, which is why a download reproduces exactly what
        // was uploaded even when the executor had to translate them to fly.
        val current = missions.currentIndex()
        items.forEach { item ->
            sendOnce(
                MavlinkMsgId.MISSION_ITEM_INT,
                MavlinkMessages.missionItemInt(
                    item, request.senderSystem, request.senderComponent, item.seq == current
                )
            )
        }
    }

    /** Progress from the executor, turned into the messages a ground station watches. */
    private val missionProgress = object : MissionProgressListener {
        override fun onItemStarted(seq: Int) {
            missions.setCurrent(seq)
        }

        override fun onItemReached(seq: Int) {
            sendOnce(MavlinkMsgId.MISSION_ITEM_REACHED, MavlinkMessages.missionItemReached(seq))
        }

        override fun onMissionFinished(completed: Boolean) {
            missions.setState(
                if (completed) MissionState.COMPLETE else MissionState.PAUSED
            )
            Log.i(TAG, "Mission finished: completed=$completed")
        }
    }

    /**
     * Answer a parameter or plan list request.
     *
     * This exists because of something only discovered by running QGroundControl against the
     * endpoint: `QGCCameraManager::_mavlinkMessageReceived` returns immediately while
     * `_initialConnectComplete` is false, so until the initial-connect state machine finishes,
     * **every camera heartbeat is discarded and the video stream is never discovered.** That state
     * machine blocks on the parameter and plan downloads. Answering them is therefore not
     * housekeeping — it is the prerequisite for video.
     *
     * Neither reply changes vehicle state: one publishes read-only values, the other says the
     * plan is empty.
     */
    private fun handleListRequest(request: MavlinkListRequest) {
        when (request.messageId) {
            MavlinkMsgId.PARAM_REQUEST_LIST -> sendParameterList()
            MavlinkMsgId.MISSION_REQUEST_LIST -> sendMissionList(request)
        }
    }

    /**
     * Stick input.
     *
     * Deliberately silent: MANUAL_CONTROL is a stream, not a command, and MAVLink defines no ack
     * for it. Answering each frame would put a reply on the wire for every stick sample.
     */
    private fun handleManualControl(control: MavlinkManualControl) {
        motionSink?.manualControl(control.roll, control.pitch, control.throttle, control.yaw)
    }

    /**
     * A parameter write.
     *
     * MAVLink's contract is that the vehicle answers with a PARAM_VALUE carrying what the
     * parameter now holds — which is how a ground station learns a write was clamped or refused.
     * Re-reading the published list rather than echoing the requested value is what makes that
     * answer honest: a refused write reports the old value, and the station sees it did not take.
     */
    private fun handleParamSet(request: MavlinkParamSet) {
        val sink = commandSink
        if (sink == null) {
            Log.d(TAG, "Refusing parameter write ${request.name}: commands disabled")
            return
        }
        val result = runCatching { sink.setParameter(request.name, request.value) }
            .getOrElse { error ->
                Log.w(TAG, "Parameter write ${request.name} failed: ${error.message}", error)
                CommandResult(MavlinkCommandOutcome.FAILED)
            }
        Log.i(TAG, "Parameter write ${request.name}=${request.value} -> ${result.outcome}")

        val parameters = runCatching { parameterProvider() }.getOrDefault(emptyList())
        val index = parameters.indexOfFirst { it.first == request.name }
        if (index < 0) {
            // Not a published parameter, so there is no current value to report back.
            return
        }
        sendOnce(
            MavlinkMsgId.PARAM_VALUE,
            MavlinkMessages.paramValue(
                parameters[index].first, parameters[index].second, parameters.size, index
            )
        )
    }

    private fun sendParameterList() {
        val parameters = runCatching { parameterProvider() }.getOrDefault(emptyList())
        if (parameters.isEmpty()) {
            Log.w(TAG, "No parameters to publish; a ground station may not complete its connect")
            return
        }
        parameters.forEachIndexed { index, (name, value) ->
            sendOnce(
                MavlinkMsgId.PARAM_VALUE,
                MavlinkMessages.paramValue(name, value, parameters.size, index)
            )
        }
        Log.i(TAG, "Published ${parameters.size} parameters")
    }

    /**
     * Serve one message on request. The set is small and explicit — a message not listed here is
     * refused rather than silently ignored, so a ground station gets an answer instead of a
     * retry loop.
     *
     * AUTOPILOT_VERSION is here because QGroundControl's initial-connect state machine asks for it
     * by name and retries until it gives up; streaming it on a timer is not enough. That was found
     * by running QGC against this endpoint and reading "RequestAutopilotVersion: Max retries
     * exhausted" in its log.
     */
    private fun sendRequestedMessage(
        messageId: Int,
        forCamera: Boolean,
        modeIndex: Int = 0
    ): Int = when {
        messageId == MavlinkMsgId.AUTOPILOT_VERSION -> {
            sendOnce(MavlinkMsgId.AUTOPILOT_VERSION, MavlinkMessages.autopilotVersion())
            Mav.RESULT_ACCEPTED
        }

        messageId == MavlinkMsgId.HOME_POSITION -> {
            val snapshot = runCatching { snapshotProvider() }.getOrDefault(MavlinkSnapshot())
            if (snapshot.homeSet) {
                sendOnce(MavlinkMsgId.HOME_POSITION, MavlinkMessages.homePosition(snapshot))
                Mav.RESULT_ACCEPTED
            } else {
                // Honest refusal beats a fabricated home point.
                Mav.RESULT_DENIED
            }
        }

        messageId == MavlinkMsgId.CAMERA_INFORMATION && forCamera -> sendCameraInformation()

        messageId == MavlinkMsgId.VIDEO_STREAM_INFORMATION && forCamera ->
            sendVideoStreamInformation()

        messageId == MavlinkMsgId.AVAILABLE_MODES -> sendAvailableModes(modeIndex)

        messageId == MavlinkMsgId.CURRENT_MODE -> {
            val snapshot = runCatching { snapshotProvider() }.getOrDefault(MavlinkSnapshot())
            sendOnce(
                MavlinkMsgId.CURRENT_MODE,
                MavlinkMessages.currentMode(
                    MavlinkFlightMode.fromDjiMode(
                        snapshot.flightMode, snapshot.manualOverrideActive
                    )
                )
            )
            Mav.RESULT_ACCEPTED
        }

        messageId == MavlinkMsgId.CAMERA_SETTINGS && forCamera -> sendCameraSettings()

        messageId == MavlinkMsgId.CAMERA_CAPTURE_STATUS && forCamera -> sendCameraCaptureStatus()

        messageId == MavlinkMsgId.VIDEO_STREAM_STATUS && forCamera -> sendVideoStreamStatus()

        messageId == MavlinkMsgId.STORAGE_INFORMATION && forCamera -> sendStorageInformation()

        else -> {
            Log.d(TAG, "Refusing request for message $messageId")
            Mav.RESULT_UNSUPPORTED
        }
    }

    private fun sendCameraInformation(): Int {
        val snapshot = runCatching { snapshotProvider() }.getOrDefault(MavlinkSnapshot())
        sendOnce(
            MavlinkMsgId.CAMERA_INFORMATION,
            MavlinkMessages.cameraInformation(
                timeBootMs = timeBootMs(),
                vendorName = CAMERA_VENDOR,
                modelName = snapshot.droneName.ifBlank { CAMERA_MODEL_FALLBACK }
            ),
            fromCamera = true
        )
        return Mav.RESULT_ACCEPTED
    }

    /**
     * Complete QGroundControl's camera model.
     *
     * These two are not optional extras. QGC requests both while bringing a camera up, retries six
     * times, and gives up — leaving its camera object null. Its PhotoVideoControl QML then reads
     * `capturesVideo`, `cameraMode`, `storageFreeStr` and `storageStatus` off that null and throws,
     * which breaks the controls around it including the map/video swap button. Advertising a
     * camera therefore commits us to answering these, even when the honest answer is "nothing to
     * report".
     */
    private fun sendCameraCaptureStatus(): Int {
        val snapshot = runCatching { snapshotProvider() }.getOrDefault(MavlinkSnapshot())
        sendOnce(
            MavlinkMsgId.CAMERA_CAPTURE_STATUS,
            MavlinkMessages.cameraCaptureStatus(
                timeBootMs(), snapshot.isRecording, capturing, imageCount.get()
            ),
            fromCamera = true
        )
        return Mav.RESULT_ACCEPTED
    }

    private fun sendCameraSettings(): Int {
        sendOnce(
            MavlinkMsgId.CAMERA_SETTINGS,
            MavlinkMessages.cameraSettings(timeBootMs(), zoomLevel = 1f),
            fromCamera = true
        )
        return Mav.RESULT_ACCEPTED
    }

    /**
     * Enumerate the mode list. `param2` of the request selects one 1-based index, or 0 for all.
     */
    private fun sendAvailableModes(requestedIndex: Int): Int {
        val modes = MavlinkFlightMode.ADVERTISED
        val selected = if (requestedIndex <= 0) {
            modes.indices.toList()
        } else {
            listOf(requestedIndex - 1).filter { it in modes.indices }
        }
        if (selected.isEmpty()) return Mav.RESULT_DENIED
        selected.forEach { i ->
            sendOnce(
                MavlinkMsgId.AVAILABLE_MODES,
                MavlinkMessages.availableModes(modes[i], index = i + 1, total = modes.size)
            )
        }
        return Mav.RESULT_ACCEPTED
    }

    private fun sendVideoStreamStatus(): Int {
        val stream = runCatching { videoStreamProvider() }.getOrNull()
        if (stream == null || stream.uri.isBlank()) return Mav.RESULT_DENIED
        sendOnce(
            MavlinkMsgId.VIDEO_STREAM_STATUS,
            MavlinkMessages.videoStreamStatus(stream.framerate, stream.widthPx, stream.heightPx),
            fromCamera = true
        )
        return Mav.RESULT_ACCEPTED
    }

    private fun sendStorageInformation(): Int {
        sendOnce(
            MavlinkMsgId.STORAGE_INFORMATION,
            MavlinkMessages.storageInformation(timeBootMs()),
            fromCamera = true
        )
        return Mav.RESULT_ACCEPTED
    }

    /**
     * Advertise the stream, or refuse honestly when there is not one.
     *
     * Returning DENIED rather than inventing a URL matters: a ground station that is handed a dead
     * RTSP address spends a long time failing to connect to it, which is worse than being told
     * there is no stream.
     */
    private fun sendVideoStreamInformation(): Int {
        val stream = runCatching { videoStreamProvider() }.getOrNull()
        if (stream == null || stream.uri.isBlank()) {
            Log.i(TAG, "No video stream to advertise yet")
            return Mav.RESULT_DENIED
        }
        sendOnce(
            MavlinkMsgId.VIDEO_STREAM_INFORMATION,
            MavlinkMessages.videoStreamInformation(
                uri = stream.uri,
                name = stream.name,
                framerate = stream.framerate,
                widthPx = stream.widthPx,
                heightPx = stream.heightPx
            ),
            fromCamera = true
        )
        Log.i(TAG, "Advertised video stream ${stream.uri}")
        return Mav.RESULT_ACCEPTED
    }

    private fun sendOnce(messageId: Int, payload: ByteArray, fromCamera: Boolean = false) {
        val bound = socket ?: return
        if (bound.isClosed) return
        val frame = if (fromCamera) {
            cameraFramer.frame(messageId, payload)
        } else {
            framer.frame(messageId, payload)
        }
        val targets = if (messageId == MavlinkMsgId.HEARTBEAT) {
            heartbeatDestinations()
        } else {
            dataDestinations()
        }
        for (destination in targets) {
            runCatching {
                bound.send(DatagramPacket(frame, frame.size, destination))
            }.onFailure { error ->
                if (error is IOException) Log.d(TAG, "Send to $destination failed: ${error.message}")
            }
        }
    }

    /**
     * Where a data frame goes: the configured GCS plus every peer that has contacted us. With
     * neither, fall back to subnet broadcast, which is how a GCS finds the aircraft without any
     * configuration at all.
     */
    private fun dataDestinations(): List<InetSocketAddress> {
        val targets = LinkedHashSet<InetSocketAddress>()
        configuredTarget?.let { targets.add(it) }
        targets.addAll(discoveredTargets)
        if (targets.isEmpty()) targets.add(broadcastTarget())
        return targets.toList()
    }

    /**
     * Heartbeats always broadcast, so a ground station that has never spoken to us can still find
     * the drone even after other peers are known.
     */
    private fun heartbeatDestinations(): List<InetSocketAddress> {
        val targets = LinkedHashSet<InetSocketAddress>()
        targets.add(broadcastTarget())
        configuredTarget?.let { targets.add(it) }
        targets.addAll(discoveredTargets)
        return targets.toList()
    }

    private fun broadcastTarget(): InetSocketAddress =
        InetSocketAddress(InetAddress.getByName(BROADCAST_ADDRESS), config.targetPort)

    private fun timeBootMs(): Long = (System.nanoTime() - bootNanos) / NANOS_PER_MILLI

    private fun unixTimeUsec(): Long = System.currentTimeMillis() * MICROS_PER_MILLI

    /** One streamed message and its interval. */
    private class Stream(
        val messageId: Int,
        private val intervalMs: Long,
        /** Send from the camera component rather than the autopilot component. */
        val camera: Boolean = false,
        /** Skip this tick entirely when the snapshot has nothing truthful to report. */
        private val sendIf: (MavlinkSnapshot) -> Boolean = { true },
        private val builder: (MavlinkSnapshot) -> ByteArray
    ) {
        fun shouldSend(snapshot: MavlinkSnapshot): Boolean = sendIf(snapshot)

        private var accumulatedMs = Long.MAX_VALUE / 2

        fun due(elapsedMs: Long): Boolean {
            accumulatedMs += elapsedMs
            if (accumulatedMs < intervalMs) return false

            // Carry the overshoot rather than discarding it. The loop tick is not exact — a
            // 20ms sleep plus the work in the loop lands nearer 24ms — so resetting to zero threw
            // away a few milliseconds every cycle. Against a 100ms interval that compounded into
            // firing every ~120ms, which measured as 8.45Hz on the aircraft against a 10Hz
            // target. Subtracting keeps the long-run average on the interval.
            accumulatedMs -= intervalMs

            // If the loop stalled for longer than an interval, drop the missed slots instead of
            // firing a burst to catch up: stale telemetry sent back-to-back is worse than a gap.
            if (accumulatedMs > intervalMs) accumulatedMs = 0
            return true
        }

        fun build(snapshot: MavlinkSnapshot): ByteArray = builder(snapshot)
    }

    companion object {
        private const val TAG = "MavlinkEndpoint"
        private const val TICK_MS = 20L
        private const val NANOS_PER_MILLI = 1_000_000L
        private const val MICROS_PER_MILLI = 1_000L
        private const val JOIN_TIMEOUT_MS = 1_000L
        private const val RECEIVE_BUFFER_BYTES = 512
        private const val BROADCAST_ADDRESS = "255.255.255.255"

        private const val HEARTBEAT_INTERVAL_MS = 1_000L
        private const val SLOW_INTERVAL_MS = 1_000L
        private const val HOME_INTERVAL_MS = 2_000L
        private const val POSITION_INTERVAL_MS = 200L
        private const val ATTITUDE_INTERVAL_MS = 100L
        private const val VERSION_INTERVAL_MS = 5_000L
        private const val CURRENT_MODE_INTERVAL_MS = 2_000L
        private const val MISSION_CURRENT_INTERVAL_MS = 1_000L
        private const val CAPTURE_STATUS_INTERVAL_MS = 1_000L

        private const val CAMERA_VENDOR = "WildBridge"
        private const val CAMERA_MODEL_FALLBACK = "DJI Camera"
    }
}
