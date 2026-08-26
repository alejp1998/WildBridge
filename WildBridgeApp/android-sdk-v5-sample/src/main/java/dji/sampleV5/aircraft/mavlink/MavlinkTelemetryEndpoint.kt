package dji.sampleV5.aircraft.mavlink

import android.util.Log
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import kotlin.concurrent.thread

/**
 * MAVLink 2 telemetry endpoint: streams the phase-1 message set over UDP so a stock ground station
 * shows the aircraft with no plugin and no configuration file.
 *
 * Inbound traffic is handled, but only just: a datagram can teach the endpoint where a ground
 * station lives, and it can ask for the camera and video-stream descriptions. Every other command
 * is acknowledged `MAV_RESULT_UNSUPPORTED` and dropped. **No inbound message can reach a flight
 * control path, because no such path is wired up here** — that is what keeps this reviewable
 * without a flight-safety argument, and it is a property to preserve deliberately rather than
 * lose by accident. See [handleCommand].
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
    private val commandSink: MavlinkCommandSink? = null
) {
    private val framer = MavlinkFramer(config.systemId)

    /**
     * The camera is a separate MAVLink component, which is not decoration: QGroundControl only
     * looks for cameras on component ids 100..105, so a stream advertised from the autopilot
     * component is never discovered.
     */
    private val cameraFramer = MavlinkFramer(config.systemId, Mav.COMP_ID_CAMERA)
    private val bootNanos = System.nanoTime()

    private var socket: DatagramSocket? = null
    private var senderThread: Thread? = null
    private var receiverThread: Thread? = null

    @Volatile
    private var running = false

    /** Explicitly configured destination, if any. */
    @Volatile
    private var configuredTarget: InetSocketAddress? = null

    /** Address of a peer that contacted us, used when no destination was configured. */
    @Volatile
    private var discoveredTarget: InetSocketAddress? = null

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
                MavlinkInbound.parseListRequest(buffer, packet.length)?.let(::handleListRequest)
            }
        }
    }

    private fun notePeer(address: InetSocketAddress) {
        if (discoveredTarget == address) return
        discoveredTarget = address
        val label = "${address.address.hostAddress}:${address.port}"
        peerAddress = label
        Log.i(TAG, "Ground station discovered at $label")
        onPeerDiscovered?.invoke(label)
    }

    private fun streamLoop() {
        val streams = buildStreams()
        var lastTick = System.nanoTime()

        while (running) {
            val now = System.nanoTime()
            val elapsedMs = (now - lastTick) / NANOS_PER_MILLI
            lastTick = now

            val snapshot = runCatching { snapshotProvider() }.getOrElse {
                Log.w(TAG, "Snapshot failed: ${it.message}")
                MavlinkSnapshot()
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

        val result = when (command.command) {
            Mav.CMD_REQUEST_MESSAGE -> sendRequestedMessage(
                command.param1.toInt(), forCamera, command.param2.toInt()
            )
            Mav.CMD_REQUEST_CAMERA_INFORMATION ->
                if (forCamera) sendCameraInformation() else Mav.RESULT_UNSUPPORTED
            Mav.CMD_REQUEST_VIDEO_STREAM_INFORMATION ->
                if (forCamera) sendVideoStreamInformation() else Mav.RESULT_UNSUPPORTED
            Mav.CMD_REQUEST_CAMERA_SETTINGS ->
                if (forCamera) sendCameraSettings() else Mav.RESULT_UNSUPPORTED
            Mav.CMD_REQUEST_STORAGE_INFORMATION ->
                if (forCamera) sendStorageInformation() else Mav.RESULT_UNSUPPORTED
            Mav.CMD_REQUEST_VIDEO_STREAM_STATUS ->
                if (forCamera) sendVideoStreamStatus() else Mav.RESULT_UNSUPPORTED
            Mav.CMD_REQUEST_CAMERA_CAPTURE_STATUS ->
                if (forCamera) sendCameraCaptureStatus() else Mav.RESULT_UNSUPPORTED
            else -> executeCommand(command)
        }

        val ack = MavlinkMessages.commandAck(
            command = command.command,
            result = result,
            targetSystem = command.senderSystem,
            targetComponent = command.senderComponent
        )
        // The ack must come from the component that was addressed, or the requester will not
        // match it to its request.
        sendOnce(MavlinkMsgId.COMMAND_ACK, ack, fromCamera = forCamera)
    }

    /**
     * Execute one payload or camera command, or refuse it.
     *
     * This is the entire set of commands that reach the aircraft, and it is an allowlist: a
     * command not named here gets `MAV_RESULT_UNSUPPORTED` and nothing happens. **No flight
     * motion is reachable from here** — takeoff, land, return, reposition, yaw and altitude are
     * not in this `when`, and are not in [MavlinkCommandSink] either, so there is no code path
     * from an inbound packet to the aircraft moving. Keeping that true is the point; adding a
     * motion command is a deliberate act with its own gate, not a new branch here.
     */
    private fun executeCommand(command: MavlinkCommand): Int {
        val sink = commandSink
        if (sink == null) {
            Log.d(TAG, "Refusing command ${command.command}: no command sink configured")
            return Mav.RESULT_UNSUPPORTED
        }
        val outcome = runCatching {
            when (command.command) {
                // param1 pitch, param2 yaw, both degrees.
                Mav.CMD_DO_GIMBAL_MANAGER_PITCHYAW ->
                    sink.setGimbalPitchYaw(command.param1, command.param2)

                // param2 is the zoom value for MAV_ZOOM_TYPE_RANGE / _CONTINUOUS.
                Mav.CMD_SET_CAMERA_ZOOM -> sink.setCameraZoom(command.param2)

                Mav.CMD_VIDEO_START_CAPTURE -> sink.startVideoRecording()
                Mav.CMD_VIDEO_STOP_CAPTURE -> sink.stopVideoRecording()
                Mav.CMD_IMAGE_START_CAPTURE -> sink.captureImage()

                else -> {
                    Log.d(TAG, "Refusing unsupported command ${command.command}")
                    MavlinkCommandOutcome.UNSUPPORTED
                }
            }
        }.getOrElse { error ->
            Log.w(TAG, "Command ${command.command} failed: ${error.message}", error)
            MavlinkCommandOutcome.FAILED
        }
        if (outcome != MavlinkCommandOutcome.UNSUPPORTED) {
            Log.i(TAG, "Command ${command.command} -> $outcome")
        }
        return outcome.mavResult
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
            MavlinkMsgId.MISSION_REQUEST_LIST -> sendOnce(
                MavlinkMsgId.MISSION_COUNT,
                MavlinkMessages.missionCount(
                    targetSystem = request.senderSystem,
                    targetComponent = request.senderComponent,
                    missionType = request.missionType
                )
            )
        }
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
        for (destination in destinations()) {
            runCatching {
                bound.send(DatagramPacket(frame, frame.size, destination))
            }.onFailure { error ->
                if (error is IOException) Log.d(TAG, "Send to $destination failed: ${error.message}")
            }
        }
    }

    /**
     * Where a frame goes. A configured host always receives; a discovered peer also receives, so
     * that a ground station on an unknown address still works. With neither, fall back to subnet
     * broadcast, which is how a GCS finds the aircraft without any configuration at all.
     */
    private fun destinations(): List<InetSocketAddress> {
        val targets = LinkedHashSet<InetSocketAddress>()
        configuredTarget?.let { targets.add(it) }
        discoveredTarget?.let { targets.add(it) }
        if (targets.isEmpty()) {
            runCatching {
                targets.add(
                    InetSocketAddress(InetAddress.getByName(BROADCAST_ADDRESS), config.targetPort)
                )
            }
        }
        return targets.toList()
    }

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
            accumulatedMs = 0
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
        private const val CAPTURE_STATUS_INTERVAL_MS = 1_000L

        private const val CAMERA_VENDOR = "WildBridge"
        private const val CAMERA_MODEL_FALLBACK = "DJI Camera"
    }
}
