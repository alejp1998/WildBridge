package dji.sampleV5.aircraft

import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.et.create
import dji.v5.et.get

enum class DroneControlProfile(
    val displayName: String,
    private val speedLimits: DroneSpeedLimits,
    private val distancePid: DronePidGains,
    private val yawControl: DroneYawControl
) {
    MAVIC_3_ENTERPRISE(
        displayName = "Mavic 3 Enterprise",
        speedLimits = DroneSpeedLimits(
            maxHorizontalSpeedMps = 15.0,
            maxGotoWpSpeedMps = 5.0,
            defaultCruiseSpeedMps = 5.0
        ),
        distancePid = DronePidGains(kp = 0.65, ki = 0.0001, kd = 0.001),
        yawControl = DroneYawControl(kp = 3.0, maxYawRateDegS = 30.0)
    ),
    MATRICE_350_RTK(
        displayName = "Matrice 350 RTK",
        speedLimits = DroneSpeedLimits(
            maxHorizontalSpeedMps = 3.0,
            maxGotoWpSpeedMps = 3.0,
            defaultCruiseSpeedMps = 3.0
        ),
        distancePid = DronePidGains(kp = 0.34, ki = 0.0001, kd = 0.001),
        yawControl = DroneYawControl(kp = 3.0, maxYawRateDegS = 30.0)
    ),
    MINI_4_PRO(
        displayName = "DJI Mini 4 Pro",
        speedLimits = DroneSpeedLimits(
            maxHorizontalSpeedMps = 15.0,
            maxGotoWpSpeedMps = 5.0,
            defaultCruiseSpeedMps = 2.0
        ),
        distancePid = DronePidGains(kp = 0.65, ki = 0.0001, kd = 0.001),
        yawControl = DroneYawControl(kp = 3.0, maxYawRateDegS = 30.0)
    );

    val maxHorizontalSpeedMps: Double get() = speedLimits.maxHorizontalSpeedMps
    val maxGotoWpSpeedMps: Double get() = speedLimits.maxGotoWpSpeedMps
    val defaultCruiseSpeedMps: Double get() = speedLimits.defaultCruiseSpeedMps
    val distanceKp: Double get() = distancePid.kp
    val distanceKi: Double get() = distancePid.ki
    val distanceKd: Double get() = distancePid.kd
    val yawKp: Double get() = yawControl.kp
    val maxYawRateDegS: Double get() = yawControl.maxYawRateDegS
}

private data class DroneSpeedLimits(
    val maxHorizontalSpeedMps: Double,
    val maxGotoWpSpeedMps: Double,
    val defaultCruiseSpeedMps: Double
)

private data class DronePidGains(
    val kp: Double,
    val ki: Double,
    val kd: Double
)

private data class DroneYawControl(
    val kp: Double,
    val maxYawRateDegS: Double
)

object DroneControlProfiles {
    fun activeProfile(): DroneControlProfile {
        val detected = ProductKey.KeyProductType.create().get(ProductType.UNKNOWN)
        return fromProductType(detected)
    }

    fun fromProductType(productType: ProductType?): DroneControlProfile {
        val name = productType?.name.orEmpty()
        return when {
            name.contains("M350", ignoreCase = true) ||
            name.contains("MATRICE_350", ignoreCase = true) -> DroneControlProfile.MATRICE_350_RTK

            name.contains("MINI_4", ignoreCase = true) ||
            name.contains("MINI4", ignoreCase = true) -> DroneControlProfile.MINI_4_PRO

            name.contains("MAVIC_3", ignoreCase = true) ||
            name.contains("MAVIC3", ignoreCase = true) ||
            name.contains("M3E", ignoreCase = true) ||
            name.contains("WM265", ignoreCase = true) -> DroneControlProfile.MAVIC_3_ENTERPRISE

            else -> DroneControlProfile.MAVIC_3_ENTERPRISE
        }
    }
}
