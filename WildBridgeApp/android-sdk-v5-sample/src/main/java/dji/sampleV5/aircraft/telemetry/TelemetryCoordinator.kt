package dji.sampleV5.aircraft.telemetry

/**
 * Thread-safe telemetry coordinator.
 * Manages the caching and formatting of real-time telemetry data.
 * Stay decoupled from DJI SDK class dependencies by holding SDK variables as [Any?].
 */
class TelemetryCoordinator {
    @Volatile var isMockEnabled: Boolean = false
    @Volatile var mockSnapshot: MockTelemetrySnapshot? = null
    @Volatile var droneName: String = "drone_1"
    
    // Position & Attitude
    @Volatile var speed: Any? = null
    @Volatile var heading: Double = 0.0
    @Volatile var attitude: Any? = null
    @Volatile var location: Any? = null
    @Volatile var altitudeASL: Double = 0.0
    @Volatile var altitudeAGL: Double = 0.0
    @Volatile var gimbalAttitude: Any? = null
    @Volatile var gimbalJointAttitude: Any? = null
    
    // Battery & Satellites
    @Volatile var batteryLevel: Int = -1
    @Volatile var satelliteCount: Int = -1
    
    // Flight & Mission
    @Volatile var homeLocation: Any? = null
    @Volatile var distanceToHome: Double = 0.0
    @Volatile var waypointReached: Boolean = false
    @Volatile var intermediaryWaypointReached: Boolean = false
    @Volatile var yawReached: Boolean = false
    @Volatile var altitudeReached: Boolean = false
    @Volatile var isRecording: Boolean = false
    @Volatile var homeSet: Boolean = false
    @Volatile var flightMode: String = "UNKNOWN"
    @Volatile var isManualOverrideActive: Boolean = false
    
    // Camera Zoom
    @Volatile var zoomFl: Int = -1
    @Volatile var hybridFl: Int = -1
    @Volatile var opticalFl: Int = -1
    @Volatile var zoomRatio: Double = 1.0
    
    // Battery assessment info
    @Volatile var remainingFlightTime: Int = 0
    @Volatile var timeNeededToGoHome: Int = 0
    @Volatile var timeNeededToLand: Int = 0
    @Volatile var totalTime: Int = 0
    @Volatile var maxRadiusCanFlyAndGoHome: Int = 0
    @Volatile var remainingCharge: Int = 0
    @Volatile var batteryNeededToLand: Int = 0
    @Volatile var batteryNeededToGoHome: Int = 0
    @Volatile var seriousLowBatteryThreshold: Int = 0
    @Volatile var lowBatteryThreshold: Int = 0
    
    // Phone location & status
    @Volatile var phoneLatitude: Double = 0.0
    @Volatile var phoneLongitude: Double = 0.0
    @Volatile var phoneHeading: Double = 0.0
    @Volatile var phonePressure: Float = 0.0f
    @Volatile var phoneBattery: Int = -1
    @Volatile var wifiRssi: Int = -100
    
    // WebRTC Metrics
    @Volatile var webRtcMetricsJson: String = "{}"
    
    // Detections status
    @Volatile var isDetectionsEnabled: Boolean = false
    @Volatile var isAutoSensingActive: Boolean = false
    @Volatile var edgeDetectionActive: Boolean = false
    @Volatile var detectionSource: String = "none"
    @Volatile var selectedDetectionSource: String = "none"
    @Volatile var detectionMenuLabel: String = "None"
    @Volatile var edgeModelName: String? = null
    @Volatile var edgeLabelsName: String? = null
    @Volatile var edgeConfidenceThreshold: Float? = null
    @Volatile var detectedTargetsJson: String = "[]"
    @Volatile var detectedTargetsSize: Int = 0

    // Streaming Config
    @Volatile var streamingMode: String = "webrtc"
    @Volatile var rtspPort: Int = 8554
    @Volatile var rtspUser: String = ""
    @Volatile var rtspPwd: String = ""
    @Volatile var rtmpUrl: String = ""

    @Volatile private var cachedTelemetryJson: String = "{}"

    fun getTelemetryJson(): String = cachedTelemetryJson

    fun rebuildTelemetryCache() {
        cachedTelemetryJson = buildTelemetryJson()
    }

    private fun escapeJson(value: String): String {
        return value.replace("\\", "\\\\").replace("\"", "\\\"")
    }

    fun detectionTelemetryJson(): String {
        val thresholdJson = edgeConfidenceThreshold?.toString() ?: "null"
        val modelJson = edgeModelName?.let { "\"${escapeJson(it)}\"" } ?: "null"
        val labelsJson = edgeLabelsName?.let { "\"${escapeJson(it)}\"" } ?: "null"
        val active = when (detectionSource) {
            "none" -> false
            "dji_onboard" -> isAutoSensingActive
            "yolo_on_phone" -> edgeDetectionActive
            else -> false
        }
        return """{"source":"$detectionSource","selectedSource":"$selectedDetectionSource","label":"$detectionMenuLabel","enabled":$isDetectionsEnabled,"active":$active,"count":$detectedTargetsSize,"model":$modelJson,"labels":$labelsJson,"confidenceThreshold":$thresholdJson,"targets":$detectedTargetsJson}"""
    }

    fun streamingTelemetryJson(): String {
        return """{"mode":"$streamingMode","rtspPort":$rtspPort,"rtspUser":"${escapeJson(rtspUser)}","rtspPwd":"${escapeJson(rtspPwd)}","rtmpUrl":"${escapeJson(rtmpUrl)}"}"""
    }

    fun buildTelemetryJson(): String {
        val mock = mockSnapshot
        val streamingJson = streamingTelemetryJson()
        if (isMockEnabled && mock != null) {
            val phoneLocationJson = """{"latitude":$phoneLatitude,"longitude":$phoneLongitude,"heading":$phoneHeading,"pressure":$phonePressure,"battery":$phoneBattery,"wifiRssi":$wifiRssi}"""
            val detectionsJson = detectionTelemetryJson()

            return """{"droneName":"$droneName","speed":${mock.velocity},"heading":${mock.heading},"attitude":${mock.attitude},"location":${mock.location},"phoneLocation":$phoneLocationJson,"webRtc":$webRtcMetricsJson,"detections":$detectionsJson,"streaming":$streamingJson,"gimbalAttitude":${mock.gimbalAttitude},"gimbalJointAttitude":${mock.gimbalAttitude},"zoomFl":24,"hybridFl":24,"opticalFl":24,"zoomRatio":1.0,"batteryLevel":${mock.batteryPercent},"satelliteCount":${mock.satelliteCount},"homeLocation":{"latitude":${mock.locationLatitude},"longitude":${mock.locationLongitude}},"distanceToHome":0.0,"waypointReached":false,"intermediaryWaypointReached":false,"yawReached":true,"altitudeReached":true,"isRecording":true,"homeSet":true,"remainingFlightTime":1320,"timeNeededToGoHome":45,"timeNeededToLand":18,"totalTime":63,"maxRadiusCanFlyAndGoHome":900,"remainingCharge":${mock.batteryPercent},"batteryNeededToLand":12,"batteryNeededToGoHome":18,"seriousLowBatteryThreshold":10,"lowBatteryThreshold":20,"flightMode":"${mock.flightMode}","isManualOverrideActive":false,"autoSensingActive":$isAutoSensingActive,"detectedTargets":$detectedTargetsJson}"""
        }

        val phoneLocationJson = """{"latitude":$phoneLatitude,"longitude":$phoneLongitude,"heading":$phoneHeading,"pressure":$phonePressure,"battery":$phoneBattery,"wifiRssi":$wifiRssi}"""
        val detectionsJson = detectionTelemetryJson()

        return """{"droneName":"$droneName","speed":$speed,"heading":$heading,"attitude":$attitude,"location":$location,"phoneLocation":$phoneLocationJson,"webRtc":$webRtcMetricsJson,"detections":$detectionsJson,"streaming":$streamingJson,"gimbalAttitude":$gimbalAttitude,"gimbalJointAttitude":$gimbalJointAttitude,"zoomFl":$zoomFl,"hybridFl":$hybridFl,"opticalFl":$opticalFl,"zoomRatio":$zoomRatio,"batteryLevel":$batteryLevel,"satelliteCount":$satelliteCount,"homeLocation":$homeLocation,"distanceToHome":$distanceToHome,"waypointReached":$waypointReached,"intermediaryWaypointReached":$intermediaryWaypointReached,"yawReached":$yawReached,"altitudeReached":$altitudeReached,"isRecording":$isRecording,"homeSet":$homeSet,"remainingFlightTime":$remainingFlightTime,"timeNeededToGoHome":$timeNeededToGoHome,"timeNeededToLand":$timeNeededToLand,"totalTime":$totalTime,"maxRadiusCanFlyAndGoHome":$maxRadiusCanFlyAndGoHome,"remainingCharge":$remainingCharge,"batteryNeededToLand":$batteryNeededToLand,"batteryNeededToGoHome":$batteryNeededToGoHome,"seriousLowBatteryThreshold":$seriousLowBatteryThreshold,"lowBatteryThreshold":$lowBatteryThreshold,"flightMode":"$flightMode","isManualOverrideActive":$isManualOverrideActive,"autoSensingActive":$isAutoSensingActive,"detectedTargets":$detectedTargetsJson}"""
    }
}
