package dji.sampleV5.aircraft.mavlink

import java.nio.ByteBuffer
import java.nio.ByteOrder

/** Message ids for the telemetry set WildBridge emits. */
internal object MavlinkMsgId {
    const val HEARTBEAT = 0
    const val SYS_STATUS = 1
    const val GPS_RAW_INT = 24
    const val ATTITUDE = 30
    const val GLOBAL_POSITION_INT = 33
    const val VFR_HUD = 74
    const val BATTERY_STATUS = 147
    const val AUTOPILOT_VERSION = 148
    const val HOME_POSITION = 242
    const val STATUSTEXT = 253

    // Parameter and mission protocols. QGroundControl will not finish its initial-connect state
    // machine without answers to these, and until it does, its camera manager discards every
    // message it receives — so the video stream is never discovered. Found by running QGC.
    const val PARAM_REQUEST_LIST = 21
    const val PARAM_VALUE = 22
    const val MISSION_REQUEST_LIST = 43
    const val MISSION_COUNT = 44

    // Camera component (see MavlinkCameraComponent).
    const val CAMERA_INFORMATION = 259
    const val VIDEO_STREAM_INFORMATION = 269
    const val CAMERA_SETTINGS = 260
    const val STORAGE_INFORMATION = 261
    const val VIDEO_STREAM_STATUS = 270
    const val CAMERA_CAPTURE_STATUS = 262
    const val CAMERA_IMAGE_CAPTURED = 263

    // Standard modes protocol. Without these a ground station can only show the raw custom_mode
    // integer, because MAV_AUTOPILOT_INVALID gives it no enum to look the number up in.
    const val AVAILABLE_MODES = 435
    const val CURRENT_MODE = 436

    // Command protocol. COMMAND_LONG and COMMAND_INT are received only; the endpoint answers
    // requests for messages and refuses everything else.
    const val COMMAND_INT = 75
    const val COMMAND_LONG = 76
    const val COMMAND_ACK = 77
}

/**
 * The enum values used by the telemetry set, taken from the upstream definitions.
 *
 * WildBridge is not a flight stack, so it reports [Mav.AUTOPILOT_INVALID] rather than claiming to
 * be PX4 or ArduPilot. That is the honest declaration for a component that speaks the protocol,
 * and it keeps mode names coming from WildBridge itself rather than being read through another
 * stack's enum. [Mav.AUTOPILOT_PX4] is left here as the documented fallback if ground-station
 * compatibility ever forces the issue.
 */
internal object Mav {
    const val TYPE_QUADROTOR = 2

    const val AUTOPILOT_GENERIC = 0
    const val AUTOPILOT_INVALID = 8
    const val AUTOPILOT_PX4 = 12

    const val STATE_UNINIT = 0
    const val STATE_STANDBY = 3
    const val STATE_ACTIVE = 4

    const val MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    const val MODE_FLAG_STABILIZE_ENABLED = 16
    const val MODE_FLAG_GUIDED_ENABLED = 8
    const val MODE_FLAG_MANUAL_INPUT_ENABLED = 64
    const val MODE_FLAG_SAFETY_ARMED = 128

    const val COMP_ID_AUTOPILOT1 = 1

    /**
     * Camera component id. QGroundControl only looks for cameras in the range
     * MAV_COMP_ID_CAMERA..MAV_COMP_ID_CAMERA6 (100..105), so the video stream is not discoverable
     * from any other component id.
     */
    const val COMP_ID_CAMERA = 100

    const val TYPE_CAMERA = 30

    const val RESULT_ACCEPTED = 0
    const val RESULT_DENIED = 2
    const val RESULT_UNSUPPORTED = 3

    const val CMD_SET_MESSAGE_INTERVAL = 511
    const val CMD_REQUEST_MESSAGE = 512
    const val CMD_REQUEST_CAMERA_INFORMATION = 521
    const val CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504
    const val CMD_REQUEST_VIDEO_STREAM_STATUS = 2505
    const val CMD_REQUEST_CAMERA_SETTINGS = 522
    const val CMD_REQUEST_STORAGE_INFORMATION = 525
    const val CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527
    const val CMD_DO_SET_STANDARD_MODE = 262

    // Payload and camera commands the endpoint executes. None of these can move the aircraft;
    // see MavlinkCommandSink for why that boundary is drawn where it is.
    const val CMD_SET_CAMERA_ZOOM = 531
    const val CMD_IMAGE_START_CAPTURE = 2000
    const val CMD_VIDEO_START_CAPTURE = 2500
    const val CMD_VIDEO_STOP_CAPTURE = 2501
    const val CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000

    const val RESULT_FAILED = 4

    // Claimed only because the endpoint actually implements them — see MavlinkCommandSink.
    // Advertising a capability commits us to the protocol behind it, which is how the camera
    // panel got into a broken state the first time.
    const val CAMERA_CAP_CAPTURE_VIDEO = 1L
    const val CAMERA_CAP_CAPTURE_IMAGE = 2L
    const val CAMERA_CAP_HAS_VIDEO_STREAM = 256L

    /** CAMERA_CAPTURE_STATUS video_status / image_status: 0 idle, 1 capture in progress. */
    const val CAPTURE_STATUS_IDLE = 0
    const val CAPTURE_STATUS_RUNNING = 1

    /**
     * RTSP, not WHEP. VIDEO_STREAM_TYPE_WHEP exists in the definitions, but QGroundControl's
     * VideoManager only auto-selects a source for RTSP, RTP/UDP, TCP-MPEG and MPEG-TS — a WHEP
     * stream would be advertised and then ignored. MediaMTX already republishes the WHIP ingest
     * on RTSP, so RTSP is both honest and the one that works.
     */
    const val VIDEO_STREAM_TYPE_RTSP = 0
    const val VIDEO_STREAM_ENCODING_H264 = 1
    const val VIDEO_STREAM_STATUS_RUNNING = 1

    /** CAMERA_MODE_VIDEO: WildBridge streams, it does not run a stills workflow. */
    const val CAMERA_MODE_VIDEO = 1

    /** STORAGE_STATUS_NOT_SUPPORTED: the DJI card is not exposed over MAVLink yet. */
    const val STORAGE_STATUS_NOT_SUPPORTED = 3

    const val STANDARD_MODE_NON_STANDARD = 0
    const val STANDARD_MODE_POSITION_HOLD = 1
    const val STANDARD_MODE_ORBIT = 2
    const val STANDARD_MODE_ALTITUDE_HOLD = 4
    const val STANDARD_MODE_SAFE_RECOVERY = 5
    const val STANDARD_MODE_MISSION = 6
    const val STANDARD_MODE_LAND = 7
    const val STANDARD_MODE_TAKEOFF = 8

    /** MAV_MODE_PROPERTY_NOT_USER_SELECTABLE: advertised for display, but not settable yet. */
    const val MODE_PROPERTY_NOT_USER_SELECTABLE = 2L

    const val SEVERITY_INFO = 6
    const val SEVERITY_WARNING = 4

    // Sensor bits used in SYS_STATUS. Only sensors WildBridge can actually speak to are reported;
    // advertising a sensor that is not backed by a real reading is the same mistake as claiming
    // the wrong autopilot.
    const val SENSOR_3D_GYRO = 1
    const val SENSOR_3D_ACCEL = 2
    const val SENSOR_3D_MAG = 4
    const val SENSOR_ABSOLUTE_PRESSURE = 8
    const val SENSOR_GPS = 32
    const val SENSOR_ATTITUDE_STABILIZATION = 2048
    const val SENSOR_YAW_POSITION = 4096
    const val SENSOR_Z_ALTITUDE_CONTROL = 8192
    const val SENSOR_XY_POSITION_CONTROL = 16384
    const val SENSOR_BATTERY = 33554432

    const val CAP_MISSION_INT = 4L
    const val CAP_COMMAND_INT = 8L
    const val CAP_MAVLINK2 = 8192L

    /** MAVLink wire-format version reported in HEARTBEAT. Always 3 for MAVLink 2. */
    const val MAVLINK_VERSION = 3

    /** MAV_PARAM_TYPE_REAL32. Every WildBridge parameter is a float for now. */
    const val PARAM_TYPE_REAL32 = 9
}

/**
 * Little-endian payload writer.
 *
 * Fields must be appended in MAVLink wire order — base fields sorted by descending type size
 * (stable, so declaration order breaks ties), then extension fields in declaration order. The
 * builders in [MavlinkMessages] follow the order generated from `common.xml`; changing the order
 * silently corrupts every field after the change, so the orders are commented per message there.
 */
internal class PayloadWriter(capacity: Int = MAX_PAYLOAD) {
    private val buffer: ByteBuffer =
        ByteBuffer.allocate(capacity).order(ByteOrder.LITTLE_ENDIAN)

    fun u8(value: Int) = apply { buffer.put((value and 0xFF).toByte()) }
    fun i8(value: Int) = apply { buffer.put(value.toByte()) }
    fun u16(value: Int) = apply { buffer.putShort((value and 0xFFFF).toShort()) }
    fun i16(value: Int) = apply { buffer.putShort(value.toShort()) }
    fun u32(value: Long) = apply { buffer.putInt((value and 0xFFFFFFFFL).toInt()) }
    fun i32(value: Int) = apply { buffer.putInt(value) }
    fun u64(value: Long) = apply { buffer.putLong(value) }
    fun f32(value: Float) = apply { buffer.putFloat(value) }

    fun f32Array(values: FloatArray, length: Int) = apply {
        for (i in 0 until length) buffer.putFloat(values.getOrElse(i) { 0f })
    }

    fun u16Array(value: Int, length: Int) = apply {
        repeat(length) { buffer.putShort((value and 0xFFFF).toShort()) }
    }

    /** Fixed-width char field: UTF-8, truncated to [length], zero-padded. */
    fun chars(text: String, length: Int) = apply {
        val bytes = text.toByteArray(Charsets.UTF_8)
        for (i in 0 until length) buffer.put(if (i < bytes.size) bytes[i] else 0)
    }

    fun zeros(count: Int) = apply { repeat(count) { buffer.put(0) } }

    fun build(): ByteArray = ByteArray(buffer.position()).also {
        buffer.flip()
        buffer.get(it)
    }

    companion object {
        const val MAX_PAYLOAD = 255
    }
}

/**
 * MAVLink 2 frame serialiser.
 *
 * Frame layout: `0xFD | len | incompat | compat | seq | sysid | compid | msgid[3] | payload | crc[2]`.
 *
 * MAVLink 2 only — no version-1 fallback path exists here by design. The 24-bit message id is what
 * lets a WildBridge dialect exist at all later, extension fields are how BATTERY_STATUS carries
 * `time_remaining`, and signing (not yet enabled) is what will eventually replace the
 * `X-Safety-Token` header. None of those exist in version 1.
 */
internal class MavlinkFramer(
    private val systemId: Int,
    private val componentId: Int = Mav.COMP_ID_AUTOPILOT1
) {
    private var sequence = 0

    fun frame(messageId: Int, payload: ByteArray): ByteArray {
        val crcExtra = MavlinkCrc.CRC_EXTRA[messageId]
            ?: error("No CRC_EXTRA registered for message id $messageId")

        // MAVLink 2 drops trailing zero bytes from the payload, but never all of them: a
        // zero-length payload is encoded as a single zero byte.
        var length = payload.size
        while (length > 1 && payload[length - 1] == ZERO_BYTE) length--
        if (payload.isEmpty()) length = 0

        val frame = ByteArray(HEADER_BYTES + length + CHECKSUM_BYTES)
        frame[0] = MAGIC_V2
        frame[1] = length.toByte()
        frame[2] = 0 // incompatibility flags: 0 = unsigned
        frame[3] = 0 // compatibility flags
        frame[4] = sequence.toByte()
        frame[5] = systemId.toByte()
        frame[6] = componentId.toByte()
        frame[7] = (messageId and 0xFF).toByte()
        frame[8] = ((messageId shr 8) and 0xFF).toByte()
        frame[9] = ((messageId shr 16) and 0xFF).toByte()
        System.arraycopy(payload, 0, frame, HEADER_BYTES, length)

        // Checksum covers LEN through the last payload byte, then CRC_EXTRA.
        val crc = MavlinkCrc.checksum(frame, 1, HEADER_BYTES - 1 + length, crcExtra)
        frame[HEADER_BYTES + length] = (crc and 0xFF).toByte()
        frame[HEADER_BYTES + length + 1] = ((crc shr 8) and 0xFF).toByte()

        sequence = (sequence + 1) and 0xFF
        return frame
    }

    companion object {
        /** Not `const`: `0xFD.toByte()` is not a compile-time constant in Kotlin. */
        val MAGIC_V2: Byte = 0xFD.toByte()
        private const val HEADER_BYTES = 10
        private const val CHECKSUM_BYTES = 2
        private const val ZERO_BYTE: Byte = 0
    }
}
