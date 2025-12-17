package dji.sampleV5.aircraft

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.Toast
import dji.sampleV5.aircraft.controller.DroneController
import dji.sampleV5.aircraft.models.BasicAircraftControlVM
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.sampleV5.aircraft.server.TelemetryServer
import dji.sampleV5.aircraft.webrtc.WebRTCMediaOptions
import dji.sampleV5.aircraft.webrtc.WebRTCStreamer
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.flightcontroller.LowBatteryRTHInfo
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.set
import dji.v5.manager.KeyManager
import dji.v5.ux.core.util.DataProcessor
import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity
import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.io.PrintWriter
import java.net.Inet4Address
import java.net.NetworkInterface
import java.net.ServerSocket
import java.net.Socket
import java.util.Collections
import java.util.concurrent.Executors
import kotlin.concurrent.thread
import androidx.lifecycle.ViewModelProvider

/**
 * WildBridge Default Layout Activity
 * 
 * Extends the DJI DefaultLayoutActivity to add:
 * - HTTP Command Server (port 8080) for drone control
 * - Telemetry Server (port 8081) for real-time telemetry data
 * - WebRTC Server (port 8082) for video streaming
 */
class WildBridgeDefaultLayoutActivity : DefaultLayoutActivity() {

    companion object {
        private const val TAG = "WildBridgeDefaultLayout"
        private const val HTTP_PORT = 8080
        private const val TELEMETRY_PORT = 8081
        private const val WEBRTC_PORT = 8082
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    
    // ViewModels for drone control
    private lateinit var basicAircraftControlVM: BasicAircraftControlVM
    private lateinit var virtualStickVM: VirtualStickVM
    
    // Servers
    private var httpServer: SimpleHttpServer? = null
    private var telemetryServer: TelemetryServer? = null
    private var webRTCStreamer: WebRTCStreamer? = null
    
    // Home point tracking
    private var isHomePointSetLatch = false

    // Battery and flight time data processors
    private val chargeRemainingProcessor: DataProcessor<Int> = DataProcessor.create(0)
    private val goHomeAssessmentProcessor: DataProcessor<LowBatteryRTHInfo> = DataProcessor.create(LowBatteryRTHInfo())
    private val seriousLowBatteryThresholdProcessor: DataProcessor<Int> = DataProcessor.create(0)
    private val lowBatteryThresholdProcessor: DataProcessor<Int> = DataProcessor.create(0)
    private val timeNeededToLandProcessor: DataProcessor<Int> = DataProcessor.create(0)

    // DJI Keys
    private val chargeRemainingKey = KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent)
    private val goHomeAssessmentKey = KeyTools.createKey(FlightControllerKey.KeyLowBatteryRTHInfo)
    private val seriousLowBatteryKey = KeyTools.createKey(FlightControllerKey.KeySeriousLowBatteryWarningThreshold)
    private val lowBatteryKey = KeyTools.createKey(FlightControllerKey.KeyLowBatteryWarningThreshold)
    private val timeNeededToLandKey = KeyTools.createKey(FlightControllerKey.KeyLowBatteryRTHInfo)

    private val gimbalKey: DJIKey.ActionKey<GimbalAngleRotation, EmptyMsg> = GimbalKey.KeyRotateByAngle.create()
    private val zoomKey: DJIKey<Double> = CameraKey.KeyCameraZoomRatios.create()
    private val startRecording: DJIKey.ActionKey<EmptyMsg, EmptyMsg> = CameraKey.KeyStartRecord.create()
    private val stopRecording: DJIKey.ActionKey<EmptyMsg, EmptyMsg> = CameraKey.KeyStopRecord.create()
    private val isRecordingKey: DJIKey<Boolean> = CameraKey.KeyIsRecording.create()

    private val location3DKey: DJIKey<LocationCoordinate3D> = FlightControllerKey.KeyAircraftLocation3D.create()
    private val satelliteCountKey: DJIKey<Int> = FlightControllerKey.KeyGPSSatelliteCount.create()
    private val gimbalAttitudeKey: DJIKey<Attitude> = GimbalKey.KeyGimbalAttitude.create()
    private val gimbalJointAttitudeKey: DJIKey<Attitude> = GimbalKey.KeyGimbalJointAttitude.create()
    private val compassHeadKey: DJIKey<Double> = FlightControllerKey.KeyCompassHeading.create()
    private val homeLocationKey: DJIKey<LocationCoordinate2D> = FlightControllerKey.KeyHomeLocation.create()
    private val flightSpeedKey: DJIKey<Velocity3D> = FlightControllerKey.KeyAircraftVelocity.create()
    private val attitudeKey: DJIKey<Attitude> = FlightControllerKey.KeyAircraftAttitude.create()
    private val cameraZoomFocalLengthKey: DJIKey<Int> = CameraKey.KeyCameraZoomFocalLength.create()
    private val cameraOpticalFocalLengthKey: DJIKey<Int> = CameraKey.KeyCameraOpticalZoomFocalLength.create()
    private val cameraHybridFocalLengthKey: DJIKey<Int> = CameraKey.KeyCameraHybridZoomFocalLength.create()
    private val batteryKey: DJIKey<Int> = BatteryKey.KeyChargeRemainingInPercent.create()
    private val flightModeKey: DJIKey<FlightMode> = FlightControllerKey.KeyFlightMode.create()
    private val isFlyingKey: DJIKey<Boolean> = FlightControllerKey.KeyIsFlying.create()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        // Initialize ViewModels
        basicAircraftControlVM = ViewModelProvider(this)[BasicAircraftControlVM::class.java]
        virtualStickVM = ViewModelProvider(this)[VirtualStickVM::class.java]
        
        // Initialize DroneController
        DroneController.init(basicAircraftControlVM, virtualStickVM)
        
        // Setup key listeners for telemetry
        setupKeyListeners()
        
        // Start all servers
        startServers()
        
        // Show IP address
        showServerInfo()
    }

    private fun setupKeyListeners() {
        KeyManager.getInstance().listen(chargeRemainingKey, this) { _, newValue ->
            chargeRemainingProcessor.onNext(newValue ?: 0)
        }
        KeyManager.getInstance().listen(goHomeAssessmentKey, this) { _, newValue ->
            goHomeAssessmentProcessor.onNext(newValue ?: LowBatteryRTHInfo())
        }
        KeyManager.getInstance().listen(seriousLowBatteryKey, this) { _, newValue ->
            seriousLowBatteryThresholdProcessor.onNext(newValue ?: 0)
        }
        KeyManager.getInstance().listen(lowBatteryKey, this) { _, newValue ->
            lowBatteryThresholdProcessor.onNext(newValue ?: 0)
        }
        KeyManager.getInstance().listen(timeNeededToLandKey, this) { _, newValue ->
            timeNeededToLandProcessor.onNext(newValue?.timeNeededToLand ?: 0)
        }
    }

    private fun startServers() {
        val deviceIp = getDeviceIpAddress()
        
        // Start HTTP Command Server
        if (!isPortInUse(HTTP_PORT)) {
            try {
                httpServer = SimpleHttpServer(HTTP_PORT)
                httpServer?.start()
                Log.i(TAG, "HTTP server started on $deviceIp:$HTTP_PORT")
            } catch (e: Exception) {
                Log.e(TAG, "Error starting HTTP server: ${e.message}")
            }
        } else {
            Log.w(TAG, "HTTP port $HTTP_PORT already in use")
        }

        // Start Telemetry Server
        if (!isPortInUse(TELEMETRY_PORT)) {
            try {
                telemetryServer = TelemetryServer(TELEMETRY_PORT, ::getTelemetryJson)
                telemetryServer?.start()
                Log.i(TAG, "Telemetry server started on $deviceIp:$TELEMETRY_PORT")
            } catch (e: Exception) {
                Log.e(TAG, "Error starting telemetry server: ${e.message}")
            }
        } else {
            Log.w(TAG, "Telemetry port $TELEMETRY_PORT already in use")
        }

        // Start WebRTC Server
        if (!isPortInUse(WEBRTC_PORT)) {
            try {
                webRTCStreamer = WebRTCStreamer(
                    context = this,
                    cameraIndex = ComponentIndexType.LEFT_OR_MAIN,
                    signalingPort = WEBRTC_PORT,
                    options = WebRTCMediaOptions()
                )
                webRTCStreamer?.listener = object : WebRTCStreamer.WebRTCStreamerListener {
                    override fun onServerStarted(ip: String, port: Int) {
                        Log.i(TAG, "WebRTC server started at ws://$ip:$port")
                    }
                    override fun onServerStopped() {
                        Log.i(TAG, "WebRTC server stopped")
                    }
                    override fun onServerError(error: String) {
                        Log.e(TAG, "WebRTC error: $error")
                    }
                    override fun onClientConnected(clientId: String, totalClients: Int) {
                        Log.i(TAG, "WebRTC client connected: $clientId (total: $totalClients)")
                    }
                    override fun onClientDisconnected(clientId: String, totalClients: Int) {
                        Log.i(TAG, "WebRTC client disconnected: $clientId (total: $totalClients)")
                    }
                }
                webRTCStreamer?.start()
                Log.i(TAG, "WebRTC server started on $deviceIp:$WEBRTC_PORT")
            } catch (e: Exception) {
                Log.e(TAG, "Error starting WebRTC server: ${e.message}")
            }
        } else {
            Log.w(TAG, "WebRTC port $WEBRTC_PORT already in use")
        }
    }

    private fun showServerInfo() {
        val deviceIp = getDeviceIpAddress() ?: "Unknown"
        val message = """
            WildBridge Servers Started
            IP: $deviceIp
            HTTP Commands: $HTTP_PORT
            Telemetry: $TELEMETRY_PORT
            WebRTC Video: $WEBRTC_PORT
        """.trimIndent()
        
        Toast.makeText(this, message, Toast.LENGTH_LONG).show()
        Log.i(TAG, message)
    }

    override fun onDestroy() {
        super.onDestroy()
        
        // Stop all servers
        httpServer?.stop()
        telemetryServer?.stop()
        webRTCStreamer?.stop()
        
        // Cancel key listeners
        KeyManager.getInstance().cancelListen(this)
        
        Log.i(TAG, "All servers stopped")
    }

    // ==================== Utility Methods ====================

    private fun getDeviceIpAddress(): String? {
        try {
            val interfaces = NetworkInterface.getNetworkInterfaces()
            for (networkInterface in Collections.list(interfaces)) {
                if (!networkInterface.isUp || networkInterface.isLoopback) continue
                val addresses = networkInterface.inetAddresses
                for (address in Collections.list(addresses)) {
                    if (address is Inet4Address && !address.isLoopbackAddress) {
                        return address.hostAddress
                    }
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error getting IP address: ${e.message}")
        }
        return null
    }

    private fun isPortInUse(port: Int): Boolean {
        return try {
            ServerSocket(port).close()
            false
        } catch (e: IOException) {
            true
        }
    }

    // ==================== Telemetry Data ====================

    private fun getLocation3D(): LocationCoordinate3D = location3DKey.get(LocationCoordinate3D(0.0, 0.0, 0.0))
    private fun getSatelliteCount(): Int = satelliteCountKey.get(-1)
    private fun getGimbalAttitude(): Attitude = gimbalAttitudeKey.get(Attitude(0.0, 0.0, 0.0))
    private fun getGimbalJointAttitude(): Attitude = gimbalJointAttitudeKey.get(Attitude(0.0, 0.0, 0.0))
    private fun getHeading(): Double = compassHeadKey.get(0.0)
    private fun getHomeLocation(): LocationCoordinate2D = homeLocationKey.get(LocationCoordinate2D())
    private fun getSpeed(): Velocity3D = flightSpeedKey.get(Velocity3D(0.0, 0.0, 0.0))
    private fun getAttitude(): Attitude = attitudeKey.get(Attitude(0.0, 0.0, 0.0))
    private fun getCameraZoomFocalLength(): Int = cameraZoomFocalLengthKey.get(-1)
    private fun getCameraOpticalFocalLength(): Int = cameraOpticalFocalLengthKey.get(-1)
    private fun getCameraHybridFocalLength(): Int = cameraHybridFocalLengthKey.get(-1)
    private fun getBatteryLevel(): Int = batteryKey.get(-1)
    private fun getFlightMode(): FlightMode = flightModeKey.get(FlightMode.UNKNOWN)
    private fun getTimeNeededToGoHome(): Int = goHomeAssessmentProcessor.value.timeNeededToGoHome
    private fun getTimeNeededToLand(): Int = timeNeededToLandProcessor.value

    private fun isHomeSet(): Boolean {
        if (isHomePointSetLatch) return true
        val isFlying = isFlyingKey.get(false)
        if (!isFlying) {
            val home = getHomeLocation()
            if (home.latitude != 0.0 && home.longitude != 0.0) {
                val current = getLocation3D()
                val distance = DroneController.calculateDistance(
                    current.latitude, current.longitude,
                    home.latitude, home.longitude
                )
                if (distance < 0.5) {
                    isHomePointSetLatch = true
                    return true
                }
            }
        }
        return isHomePointSetLatch
    }

    private fun getTelemetryJson(): String {
        val goHomeInfo = goHomeAssessmentProcessor.value
        val speed = getSpeed()
        val heading = getHeading()
        val attitude = getAttitude()
        val location = getLocation3D()
        val gimbalAttitude = getGimbalAttitude()
        val gimbalJointAttitude = getGimbalJointAttitude()
        val zoomFl = getCameraZoomFocalLength()
        val hybridFl = getCameraHybridFocalLength()
        val opticalFl = getCameraOpticalFocalLength()
        val zoomRatio = zoomKey.get()
        val batteryLevel = getBatteryLevel()
        val satelliteCount = getSatelliteCount()
        val homeLocation = getHomeLocation()
        val distanceToHome = DroneController.calculateDistance(
            location.latitude, location.longitude,
            homeLocation.latitude, homeLocation.longitude
        )
        val waypointReached = DroneController.isWaypointReached()
        val intermediaryWaypointReached = DroneController.isIntermediaryWaypointReached()
        val yawReached = DroneController.isYawReached()
        val altitudeReached = DroneController.isAltitudeReached()
        val isRecording = isRecordingKey.get()
        val homeSet = isHomeSet()
        val flightMode = getFlightMode().name

        val remainingCharge = chargeRemainingProcessor.value
        val batteryNeededToLand = goHomeInfo.batteryPercentNeededToLand
        val batteryNeededToGoHome = goHomeInfo.batteryPercentNeededToGoHome
        val seriousLowBatteryThreshold = seriousLowBatteryThresholdProcessor.value
        val lowBatteryThreshold = lowBatteryThresholdProcessor.value
        val remainingFlightTime = goHomeInfo.remainingFlightTime
        val timeNeededToGoHome = getTimeNeededToGoHome()
        val timeNeededToLand = getTimeNeededToLand()
        val totalTime = timeNeededToGoHome + timeNeededToLand
        val maxRadiusCanFlyAndGoHome = goHomeInfo.maxRadiusCanFlyAndGoHome

        return """{"speed":$speed,"heading":$heading,"attitude":$attitude,"location":$location,"gimbalAttitude":$gimbalAttitude,"gimbalJointAttitude":$gimbalJointAttitude,"zoomFl":$zoomFl,"hybridFl":$hybridFl,"opticalFl":$opticalFl,"zoomRatio":$zoomRatio,"batteryLevel":$batteryLevel,"satelliteCount":$satelliteCount,"homeLocation":$homeLocation,"distanceToHome":$distanceToHome,"waypointReached":$waypointReached,"intermediaryWaypointReached":$intermediaryWaypointReached,"yawReached":$yawReached,"altitudeReached":$altitudeReached,"isRecording":$isRecording,"homeSet":$homeSet,"remainingFlightTime":$remainingFlightTime,"timeNeededToGoHome":$timeNeededToGoHome,"timeNeededToLand":$timeNeededToLand,"totalTime":$totalTime,"maxRadiusCanFlyAndGoHome":$maxRadiusCanFlyAndGoHome,"remainingCharge":$remainingCharge,"batteryNeededToLand":$batteryNeededToLand,"batteryNeededToGoHome":$batteryNeededToGoHome,"seriousLowBatteryThreshold":$seriousLowBatteryThreshold,"lowBatteryThreshold":$lowBatteryThreshold,"flightMode":"$flightMode"}"""
    }

    // ==================== HTTP Server ====================

    private inner class SimpleHttpServer(private val port: Int) {
        private var serverSocket: ServerSocket? = null
        private val executor = Executors.newFixedThreadPool(10)
        @Volatile
        private var isRunning = false

        fun start() {
            if (isRunning) return
            thread {
                try {
                    serverSocket = ServerSocket(port)
                    isRunning = true
                    Log.i("SimpleHttpServer", "Server started on port $port")
                    while (isRunning && !serverSocket!!.isClosed) {
                        try {
                            val clientSocket = serverSocket!!.accept()
                            executor.submit { handleRequest(clientSocket) }
                        } catch (e: Exception) {
                            if (isRunning) {
                                Log.e("SimpleHttpServer", "Error accepting connection: ${e.message}")
                            }
                        }
                    }
                } catch (e: Exception) {
                    Log.e("SimpleHttpServer", "Server error: ${e.message}")
                }
            }
        }

        fun stop() {
            isRunning = false
            try {
                serverSocket?.close()
                executor.shutdown()
            } catch (e: Exception) {
                Log.e("SimpleHttpServer", "Error stopping server: ${e.message}")
            }
        }

        private fun handleRequest(clientSocket: Socket) {
            try {
                val reader = BufferedReader(InputStreamReader(clientSocket.getInputStream()))
                val writer = PrintWriter(OutputStreamWriter(clientSocket.getOutputStream()), true)

                val requestLine = reader.readLine() ?: return
                val parts = requestLine.split(" ")
                if (parts.size < 3) return

                val method = parts[0]
                val uri = parts[1]

                var contentLength = 0
                var line: String?
                while (reader.readLine().also { line = it } != null && line!!.isNotEmpty()) {
                    if (line!!.startsWith("Content-Length:", ignoreCase = true)) {
                        contentLength = line!!.substring(15).trim().toIntOrNull() ?: 0
                    }
                }

                var postData = ""
                if (method == "POST" && contentLength > 0) {
                    val buffer = CharArray(contentLength)
                    reader.read(buffer, 0, contentLength)
                    postData = String(buffer)
                }

                val response = handleHttpRequest(method, uri, postData)

                writer.println("HTTP/1.1 200 OK")
                writer.println("Content-Type: text/plain")
                writer.println("Content-Length: ${response.length}")
                writer.println("Access-Control-Allow-Origin: *")
                writer.println("Access-Control-Allow-Methods: GET, POST, OPTIONS")
                writer.println("Access-Control-Allow-Headers: Content-Type")
                writer.println()
                writer.print(response)
                writer.flush()
                clientSocket.close()
            } catch (e: Exception) {
                Log.e("SimpleHttpServer", "Error handling request: ${e.message}")
                try { clientSocket.close() } catch (ex: Exception) { }
            }
        }

        private fun handleHttpRequest(method: String, uri: String, postData: String): String {
            return when (method) {
                "POST" -> handlePostRequest(uri, postData)
                "OPTIONS" -> "OK"
                "GET" -> "Use POST for commands. Telemetry available on port $TELEMETRY_PORT"
                else -> "Method Not Allowed"
            }
        }

        private fun handlePostRequest(uri: String, postData: String): String {
            return try {
                Log.i("DroneServer", "POST $uri with data: $postData")
                when (uri) {
                    "/send/takeoff" -> {
                        DroneController.startTakeOff()
                        "Takeoff command sent."
                    }
                    "/send/land" -> {
                        DroneController.startLanding()
                        "Landing command sent."
                    }
                    "/send/RTH" -> {
                        DroneController.startReturnToHome()
                        "Return to home command sent."
                    }
                    "/send/stick" -> {
                        val cmd = postData.split(",")
                        val lx = cmd[0].toFloat()
                        val ly = cmd[1].toFloat()
                        val rx = cmd[2].toFloat()
                        val ry = cmd[3].toFloat()
                        DroneController.setStick(lx, ly, rx, ry)
                        "Received: leftX: $lx, leftY: $ly, rightX: $rx, rightY: $ry"
                    }
                    "/send/gimbal/pitch" -> {
                        val cmd = postData.split(",")
                        val roll = cmd[0].toDouble()
                        val pitch = cmd[1].toDouble()
                        val yaw = cmd[2].toDouble()
                        val rot = GimbalAngleRotation(
                            GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                            pitch, roll, yaw, false, true, true, 0.1, false, 0
                        )
                        gimbalKey.action(rot)
                        "Received: roll: $roll, pitch: $pitch, yaw: $yaw"
                    }
                    "/send/gimbal/yaw" -> {
                        val cmd = postData.split(",")
                        val roll = cmd[0].toDouble()
                        val pitch = cmd[1].toDouble()
                        val yaw = cmd[2].toDouble()
                        val rot = GimbalAngleRotation(
                            GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                            pitch, roll, yaw, true, true, false, 0.1, false, 0
                        )
                        gimbalKey.action(rot)
                        "Received: roll: $roll, pitch: $pitch, yaw: $yaw"
                    }
                    "/send/gotoYaw" -> {
                        val yaw = postData.split(",")[0].toDouble()
                        DroneController.gotoYaw(yaw)
                        "Received: yaw: $yaw"
                    }
                    "/send/gotoAltitude" -> {
                        val targetAltitude = postData.split(",")[0].toDouble()
                        DroneController.gotoAltitude(targetAltitude)
                        "Received: Altitude: $targetAltitude"
                    }
                    "/send/camera/zoom" -> {
                        val targetZoom = postData.toDouble()
                        zoomKey.set(targetZoom)
                        "Received: zoom: $targetZoom"
                    }
                    "/send/abortMission" -> {
                        DroneController.setStick(0.0f, 0.0f, 0.0f, 0.0f)
                        DroneController.disableVirtualStick()
                        "Received: abortMission"
                    }
                    "/send/enableVirtualStick" -> {
                        DroneController.enableVirtualStick()
                        "Received: enableVirtualStick"
                    }
                    "/send/camera/startRecording" -> {
                        startRecording.action()
                        "Received: camera start recording"
                    }
                    "/send/camera/stopRecording" -> {
                        stopRecording.action()
                        "Received: camera stop recording"
                    }
                    "/send/gotoWP" -> {
                        val cmd = postData.split(",")
                        if (cmd.size < 3) return "Invalid input. Expected format: lat,lon,alt"
                        val latitude = cmd[0].toDouble()
                        val longitude = cmd[1].toDouble()
                        val altitude = cmd[2].toDouble()
                        DroneController.gotoWP(latitude, longitude, altitude)
                        "Waypoint command received: Latitude=$latitude, Longitude=$longitude, Altitude=$altitude"
                    }
                    "/send/gotoWPwithPID" -> {
                        val cmd = postData.split(",")
                        if (cmd.size < 5) return "Invalid input. Expected format: lat,lon,alt,yaw,maxSpeed"
                        val latitude = cmd[0].toDouble()
                        val longitude = cmd[1].toDouble()
                        val altitude = cmd[2].toDouble()
                        val yaw = cmd[3].toDouble()
                        val maxSpeed = cmd[4].toDouble()
                        DroneController.navigateToWaypointWithPID(latitude, longitude, altitude, yaw, maxSpeed)
                        "Waypoint command received: Latitude=$latitude, Longitude=$longitude, Altitude=$altitude, Yaw=$yaw, MaxSpeed=$maxSpeed"
                    }
                    "/send/navigateTrajectory" -> {
                        Log.d("DroneServer", "Received trajectory data: $postData")
                        val segments = postData.split(";").map { it.trim() }.filter { it.isNotEmpty() }
                        if (segments.isEmpty()) return "Invalid input. Expected at least one waypoint and a yaw."
                        val lastSegment = segments.last().split(",").map { it.trim() }
                        if (lastSegment.size < 4) return "Invalid input. The last segment must have lat,lon,alt,yaw."
                        val finalLatitude = lastSegment[0].toDouble()
                        val finalLongitude = lastSegment[1].toDouble()
                        val finalAltitude = lastSegment[2].toDouble()
                        val finalYaw = lastSegment[3].toDouble()
                        val waypoints = mutableListOf<Triple<Double, Double, Double>>()
                        for (i in 0 until segments.size - 1) {
                            val parts = segments[i].split(",").map { it.trim() }
                            if (parts.size < 3) return "Invalid input at segment $i: expected lat,lon,alt"
                            waypoints.add(Triple(parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble()))
                        }
                        waypoints.add(Triple(finalLatitude, finalLongitude, finalAltitude))
                        Log.d("DroneServer", "Navigating trajectory with ${waypoints.size} waypoints, finalYaw: $finalYaw")
                        DroneController.navigateTrajectory(waypoints, finalYaw)
                        "Trajectory command received. Waypoints=${waypoints.size}, FinalYaw=$finalYaw"
                    }
                    "/send/navigateTrajectoryDJINative" -> {
                        val segments = postData.split(";").map { it.trim() }.filter { it.isNotEmpty() }
                        if (segments.size < 3) return "Invalid input. Need speed and at least 2 waypoints: speed;lat,lon,alt;..."
                        val trajectorySpeed = segments[0].toDoubleOrNull()
                            ?: return "Invalid input. Speed must be a number."
                        val waypoints = mutableListOf<Triple<Double, Double, Double>>()
                        for (i in 1 until segments.size) {
                            val parts = segments[i].split(",").map { it.trim() }
                            if (parts.size < 3) return "Invalid input at segment ${i - 1}: expected lat,lon,alt"
                            waypoints.add(Triple(parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble()))
                        }
                        if (waypoints.size < 2) return "Invalid input. Need at least 2 waypoints."
                        DroneController.navigateTrajectoryNative(waypoints, trajectorySpeed)
                        "DJI native mission requested with ${waypoints.size} waypoints at ${trajectorySpeed}m/s"
                    }
                    "/send/abort/DJIMission" -> {
                        DroneController.endMission()
                        "Mission stop requested"
                    }
                    "/send/setRTHAltitude" -> {
                        val altitude = postData.toIntOrNull()
                        if (altitude != null) {
                            DroneController.setRTHAltitude(altitude)
                            "RTH altitude set to $altitude m"
                        } else {
                            "Invalid altitude value"
                        }
                    }
                    else -> "Not Found: $uri"
                }
            } catch (e: Exception) {
                Log.e("DroneServer", "Error processing POST request: ${e.message}", e)
                "Error processing request: ${e.message}"
            }
        }
    }
}
