package dji.sampleV5.aircraft.controller

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

internal object WaypointControl {
    data class Acceptance(
        val distanceMeters: Double,
        val yawDegrees: Double,
        val altitudeMeters: Double
    )

    data class BodyVelocity(
        val forwardSpeed: Double,
        val lateralSpeed: Double
    )

    data class CooldownPlan(
        val waypointReached: Boolean,
        val reachedAtMs: Long,
        val stopAtWaypoint: Boolean
    )

    fun limitedSpeed(
        pidSpeed: Double,
        targetMaxSpeed: Double,
        lastCommandedSpeed: Double,
        maxSpeedStep: Double
    ): Double {
        return pidSpeed
            .coerceAtMost(targetMaxSpeed)
            .coerceAtMost(lastCommandedSpeed + maxSpeedStep)
    }

    fun bodyVelocity(targetSpeed: Double, movementDirection: Double, currentYaw: Double): BodyVelocity {
        val movementDirectionRelative = normalizeAngle(movementDirection - currentYaw)
        return BodyVelocity(
            forwardSpeed = targetSpeed * cos(Math.toRadians(movementDirectionRelative)),
            lateralSpeed = targetSpeed * sin(Math.toRadians(movementDirectionRelative))
        )
    }

    fun reachedTarget(distance: Double, yawError: Double, altitudeError: Double, acceptance: Acceptance): Boolean {
        return distance < acceptance.distanceMeters &&
            abs(yawError) < acceptance.yawDegrees &&
            abs(altitudeError) < acceptance.altitudeMeters
    }

    fun cooldownPlan(
        targetReached: Boolean,
        wasWaypointReached: Boolean,
        reachedAtMs: Long,
        nowMs: Long,
        holdCooldownMs: Long
    ): CooldownPlan {
        if (!targetReached) {
            return CooldownPlan(
                waypointReached = wasWaypointReached,
                reachedAtMs = 0L,
                stopAtWaypoint = false
            )
        }

        val firstReachedAtMs = if (wasWaypointReached) reachedAtMs else nowMs
        return CooldownPlan(
            waypointReached = true,
            reachedAtMs = firstReachedAtMs,
            stopAtWaypoint = nowMs - firstReachedAtMs >= holdCooldownMs
        )
    }

    private fun normalizeAngle(angle: Double): Double {
        var adjustedAngle = angle % 360.0
        if (adjustedAngle > 180.0) adjustedAngle -= 360.0
        if (adjustedAngle < -180.0) adjustedAngle += 360.0
        return adjustedAngle
    }
}
