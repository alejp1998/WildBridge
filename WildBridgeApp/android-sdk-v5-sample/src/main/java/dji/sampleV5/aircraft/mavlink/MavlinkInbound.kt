package dji.sampleV5.aircraft.mavlink

import java.nio.ByteBuffer
import java.nio.ByteOrder

/** A ground station asking for the whole parameter list, or for a plan. */
internal data class MavlinkListRequest(
    val messageId: Int,
    val senderSystem: Int,
    val senderComponent: Int,
    /** Plan type for MISSION_REQUEST_LIST: mission, fence or rally. Zero for parameters. */
    val missionType: Int
)

/**
 * A SET_MODE request. QGC's PX4 firmware plugin asks for Land / RTL by sending this rather than
 * DO_SET_MODE, so it is parsed and handled like a command.
 */
internal data class MavlinkSetMode(
    val targetSystem: Int,
    val baseMode: Int,
    val customMode: Int,
    val senderSystem: Int,
    val senderComponent: Int
)

/**
 * A decoded inbound command, reduced to the parts WildBridge acts on.
 *
 * Only what the endpoint answers is modelled. Everything else is identified far enough to be
 * discarded — there is no reason to interpret a ground station's own telemetry.
 */
internal data class MavlinkCommand(
    val command: Int,
    val targetSystem: Int,
    val targetComponent: Int,
    val senderSystem: Int,
    val senderComponent: Int,
    val param1: Float,
    val param2: Float,
    val param3: Float,
    val param4: Float,
    /** Latitude for position commands, degrees. */
    val param5: Float,
    /** Longitude for position commands, degrees. */
    val param6: Float,
    /** Altitude, metres. */
    val param7: Float
)

/**
 * Minimal MAVLink 2 frame reader.
 *
 * Deliberately narrow: it validates the checksum and extracts COMMAND_LONG / COMMAND_INT /
 * SET_MODE, and returns null for everything else. The endpoint that uses it refuses every command
 * outside a short allowlist, and flight motion only ever reaches the gated motion sink — see
 * [MavlinkTelemetryEndpoint].
 *
 * Signed frames are parsed and their signature ignored rather than rejected, so that a ground
 * station which has signing enabled is not silently unable to ask for the camera information.
 * Signing is not yet a trust boundary here; when it becomes one (replacing `X-Safety-Token`) this
 * is the place that has to start verifying rather than skipping.
 */
internal object MavlinkInbound {

    /** Byte offsets shared by COMMAND_LONG and COMMAND_INT payloads. */
    private const val COMMAND_ID_OFFSET = 28
    private const val TARGET_SYSTEM_OFFSET = 30
    private const val TARGET_COMPONENT_OFFSET = 31

    private const val HEADER_BYTES = 10
    private const val CHECKSUM_BYTES = 2
    private const val INCOMPAT_SIGNED = 0x01
    private const val SIGNATURE_BYTES = 13
    private const val MAX_PAYLOAD = 255
    private const val MISSION_TYPE_OFFSET = 2

    /**
     * Parse a PARAM_REQUEST_LIST or MISSION_REQUEST_LIST, or null for anything else.
     *
     * These are answered because QGroundControl's initial-connect state machine blocks on them,
     * and its camera manager ignores every message until that state machine completes — so
     * without them the video stream is never discovered. Neither request changes vehicle state.
     */
    fun parseListRequest(data: ByteArray, length: Int): MavlinkListRequest? {
        val frame = validate(data, length) ?: return null
        if (frame.messageId != MavlinkMsgId.PARAM_REQUEST_LIST &&
            frame.messageId != MavlinkMsgId.MISSION_REQUEST_LIST
        ) {
            return null
        }
        // target_system(u8), target_component(u8), then for missions an extension mission_type.
        val missionType = if (frame.payloadLength > MISSION_TYPE_OFFSET) {
            data[HEADER_BYTES + MISSION_TYPE_OFFSET].toInt() and 0xFF
        } else {
            0
        }
        return MavlinkListRequest(
            messageId = frame.messageId,
            senderSystem = data[5].toInt() and 0xFF,
            senderComponent = data[6].toInt() and 0xFF,
            missionType = missionType
        )
    }

    /** Frame-level facts shared by every parse path. */
    private data class Frame(val messageId: Int, val payloadLength: Int)

    private fun validate(data: ByteArray, length: Int): Frame? {
        if (length < HEADER_BYTES + CHECKSUM_BYTES) return null
        if (data[0] != MavlinkFramer.MAGIC_V2) return null

        val payloadLength = data[1].toInt() and 0xFF
        val incompatFlags = data[2].toInt() and 0xFF
        val signatureLength = if (incompatFlags and INCOMPAT_SIGNED != 0) SIGNATURE_BYTES else 0
        if (length < HEADER_BYTES + payloadLength + CHECKSUM_BYTES + signatureLength) return null

        val messageId = (data[7].toInt() and 0xFF) or
            ((data[8].toInt() and 0xFF) shl 8) or
            ((data[9].toInt() and 0xFF) shl 16)
        val crcExtra = MavlinkCrc.CRC_EXTRA[messageId] ?: return null
        val expected = MavlinkCrc.checksum(data, 1, HEADER_BYTES - 1 + payloadLength, crcExtra)
        val actual = (data[HEADER_BYTES + payloadLength].toInt() and 0xFF) or
            ((data[HEADER_BYTES + payloadLength + 1].toInt() and 0xFF) shl 8)
        if (expected != actual) return null

        return Frame(messageId, payloadLength)
    }

    /**
     * Parse a SET_MODE frame. Wire order is by descending type size: custom_mode(u32) first, then
     * target_system(u8) and base_mode(u8) in declaration order.
     */
    fun parseSetMode(data: ByteArray, length: Int): MavlinkSetMode? {
        val frame = validate(data, length) ?: return null
        if (frame.messageId != MavlinkMsgId.SET_MODE) return null
        val payload = ByteArray(MAX_PAYLOAD)
        System.arraycopy(data, HEADER_BYTES, payload, 0, frame.payloadLength)
        val buffer = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
        return MavlinkSetMode(
            targetSystem = payload[4].toInt() and 0xFF,
            baseMode = payload[5].toInt() and 0xFF,
            customMode = buffer.getInt(0),
            senderSystem = data[5].toInt() and 0xFF,
            senderComponent = data[6].toInt() and 0xFF
        )
    }

    /**
     * Parse one datagram. Returns null when the frame is not a command, is malformed, or fails its
     * checksum.
     */
    fun parseCommand(data: ByteArray, length: Int): MavlinkCommand? {
        val frame = validate(data, length) ?: return null
        if (frame.messageId != MavlinkMsgId.COMMAND_LONG &&
            frame.messageId != MavlinkMsgId.COMMAND_INT
        ) {
            return null
        }
        val payloadLength = frame.payloadLength

        // MAVLink 2 senders truncate trailing zero bytes, so a short payload is normal and the
        // missing bytes are zeros. Copy into a full-width buffer rather than reading past the end.
        val payload = ByteArray(MAX_PAYLOAD)
        System.arraycopy(data, HEADER_BYTES, payload, 0, payloadLength)
        val buffer = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)

        // COMMAND_LONG holds param5-7 as floats. COMMAND_INT stores x/y as int32 lat/lon in 1e7
        // degrees and z as a float, so normalise x/y back to degrees — the one motion command that
        // uses a position (DO_REPOSITION) then reads param5/6/7 uniformly regardless of encoding.
        val isCommandInt = frame.messageId == MavlinkMsgId.COMMAND_INT
        return MavlinkCommand(
            command = buffer.getShort(COMMAND_ID_OFFSET).toInt() and 0xFFFF,
            targetSystem = payload[TARGET_SYSTEM_OFFSET].toInt() and 0xFF,
            targetComponent = payload[TARGET_COMPONENT_OFFSET].toInt() and 0xFF,
            senderSystem = data[5].toInt() and 0xFF,
            senderComponent = data[6].toInt() and 0xFF,
            param1 = buffer.getFloat(0),
            param2 = buffer.getFloat(4),
            param3 = buffer.getFloat(8),
            param4 = buffer.getFloat(12),
            param5 = if (isCommandInt) buffer.getInt(16) / 1e7f else buffer.getFloat(16),
            param6 = if (isCommandInt) buffer.getInt(20) / 1e7f else buffer.getFloat(20),
            param7 = buffer.getFloat(24)
        )
    }
}
