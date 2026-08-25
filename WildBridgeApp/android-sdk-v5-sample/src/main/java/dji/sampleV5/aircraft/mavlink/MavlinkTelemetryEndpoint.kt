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
 * This is deliberately **send-only for control purposes**. Inbound datagrams are read for exactly
 * one reason — to learn the address of a ground station that found us first — and are otherwise
 * discarded. Nothing here can command the aircraft, which is what makes the phase reviewable
 * without a flight-safety argument.
 *
 * Messages are streamed at per-message rates rather than one uniform tick. A single rate is what
 * makes a link feel sluggish: attitude needs to be smooth, battery does not, and sending both at
 * the same interval gets the worst of each.
 */
internal class MavlinkTelemetryEndpoint(
    private val config: MavlinkEndpointConfig,
    private val snapshotProvider: () -> MavlinkSnapshot
) {
    private val framer = MavlinkFramer(config.systemId)
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
                if (stream.due(elapsedMs)) {
                    runCatching { sendOnce(stream.messageId, stream.build(snapshot)) }
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
            Stream(MavlinkMsgId.HOME_POSITION, HOME_INTERVAL_MS) { MavlinkMessages.homePosition(it) },
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
            }
        )
    }

    private fun sendOnce(messageId: Int, payload: ByteArray) {
        val bound = socket ?: return
        if (bound.isClosed) return
        val frame = framer.frame(messageId, payload)
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
        private val builder: (MavlinkSnapshot) -> ByteArray
    ) {
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
    }
}
