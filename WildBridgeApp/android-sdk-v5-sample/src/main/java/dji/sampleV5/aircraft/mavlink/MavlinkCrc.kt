package dji.sampleV5.aircraft.mavlink

/**
 * MAVLink checksum: CRC-16/MCRF4XX, the variant MAVLink calls "X.25".
 *
 * The checksum runs over every byte of the frame from LEN (index 1) to the end of the payload,
 * and then over one extra byte — CRC_EXTRA — derived from the message's field definition. That
 * extra byte is what makes a receiver reject a message whose sender was built against a different
 * version of the message definition, so the values must match the dialect exactly.
 *
 * The constants in [CRC_EXTRA] were generated from the upstream `common.xml`
 * (via minimal.xml → standard.xml → common.xml) rather than copied by hand; the generator was
 * validated against the canonical HEARTBEAT value of 50 before the rest were taken from it.
 */
internal object MavlinkCrc {

    /** CRC_EXTRA per message id, for the messages WildBridge emits. */
    val CRC_EXTRA: Map<Int, Int> = mapOf(
        MavlinkMsgId.HEARTBEAT to 50,
        MavlinkMsgId.SYS_STATUS to 124,
        MavlinkMsgId.GPS_RAW_INT to 24,
        MavlinkMsgId.ATTITUDE to 39,
        MavlinkMsgId.GLOBAL_POSITION_INT to 104,
        MavlinkMsgId.VFR_HUD to 20,
        MavlinkMsgId.BATTERY_STATUS to 154,
        MavlinkMsgId.AUTOPILOT_VERSION to 178,
        MavlinkMsgId.HOME_POSITION to 104,
        MavlinkMsgId.STATUSTEXT to 83
    )

    /** Accumulate one byte into a running CRC. */
    private fun accumulate(crc: Int, byte: Int): Int {
        var tmp = byte xor (crc and 0xFF)
        tmp = (tmp xor (tmp shl 4)) and 0xFF
        return ((crc ushr 8) xor (tmp shl 8) xor (tmp shl 3) xor (tmp ushr 4)) and 0xFFFF
    }

    /**
     * Checksum of [length] bytes of [data] starting at [offset], finished with [crcExtra].
     */
    fun checksum(data: ByteArray, offset: Int, length: Int, crcExtra: Int): Int {
        var crc = 0xFFFF
        for (i in offset until offset + length) {
            crc = accumulate(crc, data[i].toInt() and 0xFF)
        }
        return accumulate(crc, crcExtra and 0xFF)
    }
}
