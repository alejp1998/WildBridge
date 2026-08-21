package dji.sampleV5.aircraft

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class WildBridgeHttpCommandParserTest {
    @Test
    fun parseStickReadsFourFloatChannels() {
        assertEquals(
            WildBridgeHttpCommandParser.StickCommand(1.0f, -1.0f, 0.25f, -0.5f),
            WildBridgeHttpCommandParser.parseStick("1.0,-1.0,0.25,-0.5")
        )
    }

    @Test
    fun parseGimbalReadsRollPitchAndYaw() {
        assertEquals(
            WildBridgeHttpCommandParser.GimbalCommand(roll = 1.0, pitch = -10.0, yaw = 90.0),
            WildBridgeHttpCommandParser.parseGimbal("1.0,-10.0,90.0")
        )
    }

    @Test
    fun parseWaypointRejectsMissingAltitude() {
        val result = WildBridgeHttpCommandParser.parseWaypoint("55.0,12.0")

        assertEquals(
            WildBridgeHttpCommandParser.ParseResult.Invalid(
                "Invalid input. Expected format: lat,lon,alt"
            ),
            result
        )
    }

    @Test
    fun parseWaypointPidReadsAllFields() {
        val result = WildBridgeHttpCommandParser.parseWaypointPid("55.1,12.2,30.0,180.0,3.5")

        assertEquals(
            WildBridgeHttpCommandParser.ParseResult.Valid(
                WildBridgeHttpCommandParser.WaypointPidCommand(
                    latitude = 55.1,
                    longitude = 12.2,
                    altitude = 30.0,
                    yaw = 180.0,
                    maxSpeed = 3.5
                )
            ),
            result
        )
    }

    @Test
    fun parseTrajectoryReadsSemicolonSeparatedWaypoints() {
        val result = WildBridgeHttpCommandParser.parseTrajectory(
            "55.0,12.0,10.0; 55.1,12.1,11.0"
        )

        assertEquals(
            WildBridgeHttpCommandParser.ParseResult.Valid(
                listOf(Triple(55.0, 12.0, 10.0), Triple(55.1, 12.1, 11.0))
            ),
            result
        )
    }

    @Test
    fun parseTrajectoryReportsSegmentIndex() {
        val result = WildBridgeHttpCommandParser.parseTrajectory("55.0,12.0,10.0;55.1,12.1")

        assertEquals(
            WildBridgeHttpCommandParser.ParseResult.Invalid(
                "Invalid input at segment 1: expected lat,lon,alt"
            ),
            result
        )
    }

    @Test
    fun parseNativeTrajectoryReadsSpeedAndWaypoints() {
        val result = WildBridgeHttpCommandParser.parseNativeTrajectory(
            "2.5;55.0,12.0,10.0;55.1,12.1,11.0"
        )

        assertEquals(
            WildBridgeHttpCommandParser.ParseResult.Valid(
                WildBridgeHttpCommandParser.NativeTrajectoryCommand(
                    speed = 2.5,
                    waypoints = listOf(Triple(55.0, 12.0, 10.0), Triple(55.1, 12.1, 11.0))
                )
            ),
            result
        )
    }

    @Test
    fun parseNativeTrajectoryRejectsInvalidSpeed() {
        val result = WildBridgeHttpCommandParser.parseNativeTrajectory(
            "fast;55.0,12.0,10.0;55.1,12.1,11.0"
        )

        assertTrue(result is WildBridgeHttpCommandParser.ParseResult.Invalid)
        assertEquals(
            "Invalid input. Speed must be a number.",
            (result as WildBridgeHttpCommandParser.ParseResult.Invalid).message
        )
    }
}
