package dji.sampleV5.aircraft.controller

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class TrajectoryControlTest {
    @Test
    fun progressOnSegmentReturnsFractionAlongSegment() {
        val start = TrajectoryControl.Point(latitude = 0.0, longitude = 0.0, altitude = 10.0)
        val end = TrajectoryControl.Point(latitude = 0.0, longitude = 10.0, altitude = 20.0)

        val progress = TrajectoryControl.progressOnSegment(
            start = start,
            end = end,
            latitude = 0.0,
            longitude = 2.5
        )

        assertEquals(0.25, progress, 0.0001)
    }

    @Test
    fun progressOnSegmentReturnsZeroForZeroLengthSegment() {
        val point = TrajectoryControl.Point(latitude = 1.0, longitude = 2.0, altitude = 3.0)

        val progress = TrajectoryControl.progressOnSegment(
            start = point,
            end = point,
            latitude = 2.0,
            longitude = 3.0
        )

        assertEquals(0.0, progress, 0.0001)
    }

    @Test
    fun lookaheadRatioAddsLookaheadDistanceAndBoundsToSegment() {
        assertEquals(0.7, TrajectoryControl.lookaheadRatio(10.0, 0.5, 2.0), 0.0001)
        assertEquals(1.0, TrajectoryControl.lookaheadRatio(10.0, 0.9, 5.0), 0.0001)
        assertEquals(0.0, TrajectoryControl.lookaheadRatio(0.0, 0.5, 2.0), 0.0001)
    }

    @Test
    fun lookaheadPointInterpolatesPositionAndAltitude() {
        val start = TrajectoryControl.Point(latitude = 1.0, longitude = 2.0, altitude = 10.0)
        val end = TrajectoryControl.Point(latitude = 3.0, longitude = 6.0, altitude = 20.0)

        val point = TrajectoryControl.lookaheadPoint(start, end, ratio = 0.5)

        assertEquals(2.0, point.latitude, 0.0001)
        assertEquals(4.0, point.longitude, 0.0001)
        assertEquals(15.0, point.altitude, 0.0001)
    }

    @Test
    fun speedForSegmentSlowsOnlyInsideFinalSegmentSlowdownRadius() {
        assertEquals(3.0, TrajectoryControl.speedForSegment(5.0, 1.0, 5.0, 10.0, true), 0.0001)
        assertEquals(5.0, TrajectoryControl.speedForSegment(5.0, 1.0, 15.0, 10.0, true), 0.0001)
        assertEquals(5.0, TrajectoryControl.speedForSegment(5.0, 1.0, 5.0, 10.0, false), 0.0001)
    }

    @Test
    fun speedAdjustedForYawKeepsMinimumFactor() {
        assertEquals(5.0, TrajectoryControl.speedAdjustedForYaw(5.0, yawError = 0.0), 0.0001)
        assertEquals(2.5, TrajectoryControl.speedAdjustedForYaw(5.0, yawError = 22.5), 0.0001)
        assertEquals(1.75, TrajectoryControl.speedAdjustedForYaw(5.0, yawError = 90.0), 0.0001)
    }

    @Test
    fun reachedFinalWaypointRequiresLastSegmentDistanceAndAltitude() {
        assertTrue(TrajectoryControl.reachedFinalWaypoint(true, 1.0, 0.2, 1.5, 0.5))
        assertFalse(TrajectoryControl.reachedFinalWaypoint(false, 1.0, 0.2, 1.5, 0.5))
        assertFalse(TrajectoryControl.reachedFinalWaypoint(true, 2.0, 0.2, 1.5, 0.5))
        assertFalse(TrajectoryControl.reachedFinalWaypoint(true, 1.0, 0.8, 1.5, 0.5))
    }

    @Test
    fun planTickAcceleratesTowardLookaheadAndKeepsAltitudeSmooth() {
        val plan = TrajectoryControl.planTick(
            TrajectoryControl.TickInput(
                current = TrajectoryControl.Point(0.0, 0.0, 10.0),
                currentYaw = 90.0,
                start = TrajectoryControl.Point(0.0, 0.0, 10.0),
                end = TrajectoryControl.Point(0.0, 0.001, 20.0),
                isLastSegment = false,
                lastCommandedSpeed = 0.0,
                config = config(maxSpeedStep = 0.05)
            )
        )

        assertEquals(0.05, plan.targetSpeed, 0.0001)
        assertEquals(0.05, plan.forwardSpeed, 0.0001)
        assertEquals(0.0, plan.lateralSpeed, 0.0001)
        assertTrue(plan.targetAltitude > 10.0)
        assertFalse(plan.reached)
    }

    @Test
    fun planTickMarksFinalWaypointReachedWhenCloseToEndAndAltitude() {
        val plan = TrajectoryControl.planTick(
            TrajectoryControl.TickInput(
                current = TrajectoryControl.Point(0.0, 0.001, 20.0),
                currentYaw = 90.0,
                start = TrajectoryControl.Point(0.0, 0.0, 10.0),
                end = TrajectoryControl.Point(0.0, 0.001, 20.0),
                isLastSegment = true,
                lastCommandedSpeed = 1.0,
                config = config(acceptedDistance = 2.0, acceptedAltitudeError = 0.5)
            )
        )

        assertTrue(plan.reached)
    }

    private fun config(
        maxSpeedStep: Double = 0.5,
        acceptedDistance: Double = 1.5,
        acceptedAltitudeError: Double = 0.5
    ): TrajectoryControl.Config {
        return TrajectoryControl.Config(
            lookaheadDistance = 2.5,
            cruiseSpeed = 5.0,
            minSpeedFinal = 0.6,
            slowdownRadius = 5.0,
            maxSpeedStep = maxSpeedStep,
            acceptedDistance = acceptedDistance,
            acceptedAltitudeError = acceptedAltitudeError
        )
    }
}
