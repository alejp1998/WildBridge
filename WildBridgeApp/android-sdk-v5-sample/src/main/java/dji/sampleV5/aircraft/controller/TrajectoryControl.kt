package dji.sampleV5.aircraft.controller

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

internal object TrajectoryControl {
    data class Point(
        val latitude: Double,
        val longitude: Double,
        val altitude: Double
    ) {
        companion object {
            fun fromTriple(point: Triple<Double, Double, Double>): Point {
                return Point(point.first, point.second, point.third)
            }
        }
    }

    data class Config(
        val lookaheadDistance: Double,
        val cruiseSpeed: Double,
        val minSpeedFinal: Double,
        val slowdownRadius: Double,
        val maxSpeedStep: Double,
        val acceptedDistance: Double,
        val acceptedAltitudeError: Double
    )

    data class TickInput(
        val current: Point,
        val currentYaw: Double,
        val start: Point,
        val end: Point,
        val isLastSegment: Boolean,
        val lastCommandedSpeed: Double,
        val config: Config
    )

    data class TickPlan(
        val progress: Double,
        val targetSpeed: Double,
        val forwardSpeed: Double,
        val lateralSpeed: Double,
        val targetYawRate: Double,
        val targetAltitude: Double,
        val reached: Boolean
    )

    private const val YAW_GAIN = 1.0
    private const val MAX_YAW_RATE = 30.0

    fun progressOnSegment(start: Point, end: Point, latitude: Double, longitude: Double): Double {
        val segmentLatitude = end.latitude - start.latitude
        val segmentLongitude = end.longitude - start.longitude
        val segmentLengthSquared = segmentLatitude * segmentLatitude + segmentLongitude * segmentLongitude
        if (segmentLengthSquared == 0.0) return 0.0
        val dot = (latitude - start.latitude) * segmentLatitude + (longitude - start.longitude) * segmentLongitude
        return dot / segmentLengthSquared
    }

    fun lookaheadRatio(segmentDistance: Double, progress: Double, lookaheadDistance: Double): Double {
        if (segmentDistance <= 0.0) return 0.0
        return ((segmentDistance * progress.coerceIn(0.0, 1.0)) + lookaheadDistance)
            .div(segmentDistance)
            .coerceIn(0.0, 1.0)
    }

    fun lookaheadPoint(start: Point, end: Point, ratio: Double): Point {
        val boundedRatio = ratio.coerceIn(0.0, 1.0)
        return Point(
            latitude = start.latitude + (end.latitude - start.latitude) * boundedRatio,
            longitude = start.longitude + (end.longitude - start.longitude) * boundedRatio,
            altitude = start.altitude + (end.altitude - start.altitude) * boundedRatio
        )
    }

    fun speedForSegment(
        cruiseSpeed: Double,
        minSpeedFinal: Double,
        distanceToEnd: Double,
        slowdownRadius: Double,
        isLastSegment: Boolean
    ): Double {
        return if (isLastSegment && distanceToEnd < slowdownRadius) {
            minSpeedFinal + (cruiseSpeed - minSpeedFinal) * (distanceToEnd / slowdownRadius)
        } else {
            cruiseSpeed
        }
    }

    fun speedAdjustedForYaw(targetSpeed: Double, yawError: Double): Double {
        return targetSpeed * max(0.35, 1.0 - (abs(yawError) / 45.0))
    }

    fun reachedFinalWaypoint(
        isLastSegment: Boolean,
        distanceToEnd: Double,
        altitudeError: Double,
        acceptedDistance: Double,
        acceptedAltitudeError: Double
    ): Boolean {
        return isLastSegment &&
            distanceToEnd < acceptedDistance &&
            abs(altitudeError) < acceptedAltitudeError
    }

    fun planTick(input: TickInput): TickPlan {
        val progress = progressOnSegment(
            start = input.start,
            end = input.end,
            latitude = input.current.latitude,
            longitude = input.current.longitude
        )
        val segmentDistance = distanceMeters(input.start, input.end)
        val lookahead = lookaheadPoint(
            start = input.start,
            end = input.end,
            ratio = lookaheadRatio(segmentDistance, progress, input.config.lookaheadDistance)
        )
        val targetYaw = bearingDegrees(input.current, lookahead)
        val yawError = normalizeAngle(targetYaw - input.currentYaw)
        val targetYawRate = (YAW_GAIN * yawError).coerceIn(-MAX_YAW_RATE, MAX_YAW_RATE)
        val moveDirectionRelative = normalizeAngle(targetYaw - input.currentYaw)
        val distanceToEnd = distanceMeters(input.current, input.end)
        val targetSpeed = speedAdjustedForYaw(
            speedForSegment(
                cruiseSpeed = input.config.cruiseSpeed,
                minSpeedFinal = input.config.minSpeedFinal,
                distanceToEnd = distanceToEnd,
                slowdownRadius = input.config.slowdownRadius,
                isLastSegment = input.isLastSegment
            ),
            yawError = yawError
        ).coerceAtMost(input.lastCommandedSpeed + input.config.maxSpeedStep)

        return TickPlan(
            progress = progress,
            targetSpeed = targetSpeed,
            forwardSpeed = targetSpeed * cos(Math.toRadians(moveDirectionRelative)),
            lateralSpeed = targetSpeed * sin(Math.toRadians(moveDirectionRelative)),
            targetYawRate = targetYawRate,
            targetAltitude = lookahead.altitude,
            reached = reachedFinalWaypoint(
                isLastSegment = input.isLastSegment,
                distanceToEnd = distanceToEnd,
                altitudeError = lookahead.altitude - input.current.altitude,
                acceptedDistance = input.config.acceptedDistance,
                acceptedAltitudeError = input.config.acceptedAltitudeError
            )
        )
    }

    private fun distanceMeters(start: Point, end: Point): Double {
        val earthRadiusMeters = 6371000.0
        val phi1 = Math.toRadians(start.latitude)
        val phi2 = Math.toRadians(end.latitude)
        val deltaPhi = Math.toRadians(end.latitude - start.latitude)
        val deltaLambda = Math.toRadians(end.longitude - start.longitude)
        val haversine = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
            cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2)
        val angularDistance = 2 * atan2(sqrt(haversine), sqrt(1 - haversine))
        return earthRadiusMeters * angularDistance
    }

    private fun bearingDegrees(start: Point, end: Point): Double {
        val phi1 = Math.toRadians(start.latitude)
        val phi2 = Math.toRadians(end.latitude)
        val deltaLambda = Math.toRadians(end.longitude - start.longitude)
        val y = sin(deltaLambda) * cos(phi2)
        val x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda)
        return (Math.toDegrees(atan2(y, x)) + 360) % 360
    }

    private fun normalizeAngle(angle: Double): Double {
        var adjustedAngle = angle % 360.0
        if (adjustedAngle > 180.0) adjustedAngle -= 360.0
        if (adjustedAngle < -180.0) adjustedAngle += 360.0
        return adjustedAngle
    }
}
