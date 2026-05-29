package dji.sampleV5.aircraft

import android.content.Intent
import android.content.ContentResolver
import android.os.Build
import android.os.Bundle
import android.os.Environment
import android.os.Handler
import android.os.Looper
import android.provider.Settings
import android.provider.DocumentsContract
import java.io.File
import android.util.Log
import android.widget.Toast
import android.widget.TextView
import android.widget.ArrayAdapter
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.content.Context
import android.Manifest
import android.content.pm.PackageManager
import android.content.SharedPreferences
import android.hardware.Sensor
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.content.res.ColorStateList
import android.graphics.ImageFormat
import android.graphics.Matrix
import android.graphics.RectF
import android.graphics.SurfaceTexture
import android.widget.CheckBox
import android.media.MediaPlayer
import android.media.Image
import android.media.ImageReader
import android.widget.ImageButton
import android.widget.PopupMenu
import android.widget.Switch
import android.widget.ToggleButton
import android.widget.EditText
import android.widget.LinearLayout
import android.view.Menu
import android.view.MenuItem
import android.view.Surface
import android.view.TextureView
import android.view.View
import android.view.ViewGroup
import androidx.appcompat.app.AlertDialog
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.wifi.WifiManager
import android.net.Uri
import android.os.BatteryManager
import android.os.HandlerThread
import androidx.core.app.ActivityCompat
import dji.sampleV5.aircraft.controller.DroneController
import dji.sampleV5.aircraft.edge.EdgeDetectionController
import dji.sampleV5.aircraft.edge.EdgeDetectionController.EdgeDetectionMetrics
import dji.sampleV5.aircraft.edge.EdgeDetectionConfig
import dji.v5.ux.detection.DetectedTarget
import dji.v5.ux.detection.DetectionOverlayView
import dji.sampleV5.aircraft.logger.WildBridgeFlightLogger
import dji.sampleV5.aircraft.models.BasicAircraftControlVM
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.sampleV5.aircraft.server.TelemetryServer
import dji.sampleV5.aircraft.webrtc.WebRTCMediaOptions
import dji.sampleV5.aircraft.webrtc.WebRTCStreamer
import dji.sampleV5.aircraft.webrtc.WebRTCStreamer.VideoSourceMode
import dji.sampleV5.aircraft.webrtc.WebRTCStreamMetrics
import dji.sampleV5.aircraft.webrtc.SharedPhoneCameraFrameSource
import dji.sampleV5.aircraft.webrtc.TelemetryProvider
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sdk.keyvalue.value.camera.CameraMode
import dji.sdk.keyvalue.value.camera.CameraStorageInfos
import dji.sdk.keyvalue.value.camera.CameraStorageLocation
import dji.sdk.keyvalue.value.camera.SDCardLoadState
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.flightcontroller.LowBatteryRTHInfo
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.set
import dji.v5.manager.KeyManager
import dji.v5.ux.core.util.DataProcessor
import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity
import dji.v5.manager.intelligent.AutoSensingInfo
import dji.v5.manager.intelligent.AutoSensingInfoListener
import dji.v5.manager.intelligent.AutoSensingTarget
import dji.v5.manager.intelligent.IntelligentFlightManager
import dji.v5.manager.intelligent.IntelligentModel
import dji.v5.manager.intelligent.TargetType
import dji.v5.manager.intelligent.smarttrack.SmartTrackTarget
import dji.v5.manager.intelligent.spotlight.SpotLightTarget
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.sdk.keyvalue.value.common.DoubleRect
import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.io.PrintWriter
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.InetAddress
import java.net.MulticastSocket
import java.net.NetworkInterface
import java.net.ServerSocket
import java.net.Socket
import java.net.SocketException
import java.lang.ref.WeakReference
import java.util.Collections
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread
import androidx.lifecycle.ViewModelProvider
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo

/**
 * WildBridge Default Layout Activity
 * 
 * Extends the DJI DefaultLayoutActivity to add:
 * - HTTP Command Server (port 8080) for drone control
 * - Telemetry Server (port 8081) for real-time telemetry data
 * - WHIP publishing for WebRTC video streaming through MediaMTX
 * - mDNS/Bonjour service advertising for automatic discovery
 */
class WildBridgeDefaultLayoutActivity : DefaultLayoutActivity() {

    companion object {
        private const val TAG = "WildBridgeDefaultLayout"
        private const val HTTP_PORT = 8080
        private const val TELEMETRY_PORT = 8081
        private const val AUTONOMOUS_COMMAND_REJECTED =
            "REJECTED: Manual override active. Deactivate manual override first."
        private const val MEDIAMTX_WHIP_PORT = 8889  // mediamtx WebRTC port for WHIP publish
        private const val PREF_DRONE_NAME = "drone_name"
        private const val PREF_MEDIAMTX_SERVER = "mediamtx_server"
        private const val PREF_WEBRTC_FPS = "webrtc_fps"
        private const val PREF_WEBRTC_RESOLUTION = "webrtc_resolution"
        private const val PREF_MOCK_VIDEO_ENABLED = "mock_video_enabled"
        private const val PREF_MAP_EXPANDED = "map_expanded"
        private const val PREF_DETECTIONS_ENABLED = "detections_enabled"
        private const val PREF_DETECTION_SOURCE = "detection_source"
        private const val PREF_EDGE_DETECTION_ENABLED = "edge_detection_enabled"
        private const val PREF_VIDEO_SOURCE = "video_source"
        private const val PREF_EDGE_MODEL_URI = "edge_model_uri"
        private const val PREF_EDGE_MODEL_NAME = "edge_model_name"
        private const val PREF_EDGE_LABELS_URI = "edge_labels_uri"
        private const val PREF_EDGE_LABELS_NAME = "edge_labels_name"
        private const val PREF_EDGE_CONFIDENCE_THRESHOLD = "edge_confidence_threshold"
        private const val DEFAULT_WEBRTC_FPS = 10
        private const val DEFAULT_EDGE_CONFIDENCE_THRESHOLD = 0.25f
        private const val REQUEST_PHONE_CAMERA_SOURCE = 2
        private const val REQUEST_EDGE_MODEL_FILE = 3
        private const val REQUEST_EDGE_LABELS_FILE = 4
        private const val PHONE_EDGE_FRAME_INTERVAL_NS = 200_000_000L
        private val EDGE_CONFIDENCE_OPTIONS = floatArrayOf(0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f)
        private const val DEFAULT_DRONE_NAME = "drone_1"
        private const val DISCOVERY_PORT = 30000
        private const val DISCOVERY_MSG = "DISCOVER_WILDBRIDGE"
        private const val DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"
        private const val MULTICAST_GROUP = "239.255.42.99"
        private const val MULTICAST_PORT = 30001
        
        // mDNS/Zeroconf service type
        private const val MDNS_SERVICE_TYPE = "_wildbridge._tcp."
        private val WEBRTC_FPS_OPTIONS = intArrayOf(5, 10, 15, 20, 25, 30)
    }

    private enum class StreamResolutionPreset(
        val prefValue: String,
        val menuLabel: String,
        val width: Int,
        val height: Int,
        val bitrate: Int
    ) {
        AUTO("auto", "Auto / native", 0, 0, 6_000_000),
        FULL_HD("1080p", "1080p", 1920, 1080, 8_000_000),
        HD("720p", "720p", 1280, 720, 2_000_000),
        SD("480p", "480p", 640, 480, 1_500_000);

        companion object {
            fun fromPref(value: String?): StreamResolutionPreset {
                return entries.firstOrNull { it.prefValue == value } ?: AUTO
            }
        }
    }

    private enum class DetectionSource(
        val prefValue: String,
        val menuLabel: String
    ) {
        NONE("none", "None"),
        DJI_ONBOARD("dji_onboard", "DJI onboard"),
        YOLO_ON_PHONE("yolo_on_phone", "YOLO on phone");

        companion object {
            fun fromPref(value: String?): DetectionSource {
                return entries.firstOrNull { it.prefValue == value } ?: NONE
            }
        }
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    
    // ViewModels for drone control
    private lateinit var basicAircraftControlVM: BasicAircraftControlVM
    private lateinit var virtualStickVM: VirtualStickVM
    
    // Servers
    private var httpServer: SimpleHttpServer? = null
    private var telemetryServer: TelemetryServer? = null
    private var webRTCStreamer: WebRTCStreamer? = null
    @Volatile private var lastWhipUrl: String? = null  // Remembered for FPS/Quality mode restarts
    
    // Discovery (UDP broadcast/multicast)
    private var discoverySocket: DatagramSocket? = null
    private var multicastSocket: MulticastSocket? = null
    private var discoveryThread: Thread? = null
    private var multicastThread: Thread? = null
    private var isDiscoveryRunning = false
    private var droneSerialNumber: String = "UNKNOWN"
    
    // mDNS/Zeroconf service registration
    private var nsdManager: NsdManager? = null
    private var mdnsServiceName: String? = null
    private var isMdnsRegistered = false
    private var isMdnsRegistrationRequested = false
    
    // Drone Configuration
    private lateinit var sharedPreferences: SharedPreferences
    private var droneName: String = DEFAULT_DRONE_NAME

    // Phone Location
    private var locationManager: LocationManager? = null
    private var phoneLocation: Location? = null
    private val locationListener = object : LocationListener {
        override fun onLocationChanged(location: Location) {
            phoneLocation = location
            refreshMockTelemetryMode()
        }
        override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) = Unit
        override fun onProviderEnabled(provider: String) = Unit
        override fun onProviderDisabled(provider: String) = Unit
    }

    // Phone Sensors & Status
    private var sensorManager: SensorManager? = null
    private var wifiManager: WifiManager? = null
    private var multicastLock: WifiManager.MulticastLock? = null
    private var batteryManager: BatteryManager? = null
        private var mockPreviewPlayer: MediaPlayer? = null
    @Volatile private var lastWebRTCMetrics = WebRTCStreamMetrics()
    
    private var phoneHeading: Double = 0.0
    private var phonePressure: Float = 0.0f
    
    private val accelerometerReading = FloatArray(3)
    private val magnetometerReading = FloatArray(3)
    private val rotationMatrix = FloatArray(9)
    private val orientationAngles = FloatArray(3)
    
    private val sensorListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
                System.arraycopy(event.values, 0, accelerometerReading, 0, accelerometerReading.size)
            } else if (event.sensor.type == Sensor.TYPE_MAGNETIC_FIELD) {
                System.arraycopy(event.values, 0, magnetometerReading, 0, magnetometerReading.size)
            } else if (event.sensor.type == Sensor.TYPE_PRESSURE) {
                phonePressure = event.values[0]
            }
            
            updateOrientationAngles()
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            // Do nothing
        }
    }
    
    // Home point tracking
    private var isHomePointSetLatch = false

    // ==================== AutoSensing (AI Detection) ====================
    private var isAutoSensingActive = false
    private var isAutoSensingListenerRegistered = false
    private var edgeDetectionController: EdgeDetectionController? = null
    @Volatile private var lastEdgeMetrics = EdgeDetectionMetrics()
    @Volatile private var currentDetectedTargets: List<DetectedTarget> = emptyList()
    private var detectionOverlay: DetectionOverlayView? = null
    private var pendingVideoSourceAfterPermission: VideoSourceMode? = null
    private var phoneCameraDevice: CameraDevice? = null
    private var phoneCameraSession: CameraCaptureSession? = null
    private var phoneCameraThread: HandlerThread? = null
    private var phoneCameraHandler: Handler? = null
    private var phonePreviewSurface: Surface? = null
    private var phoneImageReader: ImageReader? = null
    private val phoneInferenceBusy = AtomicBoolean(false)
    @Volatile private var lastPhoneEdgeFrameNs = 0L

    private val autoSensingInfoListener = object : AutoSensingInfoListener {
        override fun onAutoSensingInfoUpdate(info: AutoSensingInfo) {
            if (getDetectionSource() != DetectionSource.DJI_ONBOARD) return
            val targets = info.targets?.mapIndexed { idx, t ->
                val rect = t.rect
                // DoubleRect is center-based: (x,y) = center, (width,height) = dimensions
                val cx = rect?.x ?: 0.0
                val cy = rect?.y ?: 0.0
                val hw = (rect?.width ?: 0.0) / 2.0
                val hh = (rect?.height ?: 0.0) / 2.0
                DetectedTarget(
                    index = t.targetIndex,
                    type = t.targetType?.name ?: "UNKNOWN",
                    left = cx - hw,
                    top = cy - hh,
                    right = cx + hw,
                    bottom = cy + hh
                )
            } ?: emptyList()
            applyDetectedTargets(targets)
        }

        override fun onTrackingTargetUpdate(target: AutoSensingTarget) = Unit

        override fun onIntelligentModelUpdate(models: MutableList<IntelligentModel>) = Unit

        override fun onRunningIntelligentModelUpdate(modelId: Int) = Unit
    }
    // ==================== End AutoSensing Fields ====================

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

    /**
     * Pre-built telemetry JSON string, refreshed via KeyManager listeners whenever
     * any telemetry value changes.  getTelemetryJson() just returns this cached string
     * so the TelemetryServer send loop (Thread.sleep 10 ms) does zero SDK work.
     */
    @Volatile private var cachedTelemetryJson: String = "{}"

    private val productTypeKey: DJIKey<ProductType> = ProductKey.KeyProductType.create()
    private val flightControllerConnectionKey: DJIKey<Boolean> = FlightControllerKey.KeyConnection.create()
    private val cameraModeKey: DJIKey<CameraMode> = KeyTools.createKey(CameraKey.KeyCameraMode, ComponentIndexType.LEFT_OR_MAIN)
    private val cameraStorageLocationKey: DJIKey<CameraStorageLocation> = KeyTools.createKey(CameraKey.KeyCameraStorageLocation, ComponentIndexType.LEFT_OR_MAIN)
    private val cameraStorageInfosKey: DJIKey<CameraStorageInfos> = KeyTools.createKey(CameraKey.KeyCameraStorageInfos, ComponentIndexType.LEFT_OR_MAIN)

    private data class DroneStorageStatus(
        val label: String,
        val summary: String
    ) {
        val menuLabel: String
            get() = "$label (${summary})"

        val dialogText: String
            get() = "$label: $summary"
    }

    private data class SettingsActionRow(
        val title: String,
        val detail: String? = null,
        val enabled: Boolean = true
    )

    @Volatile
    private var aircraftConnected = true

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        // Initialize SharedPreferences
        sharedPreferences = getSharedPreferences("WildBridgePrefs", Context.MODE_PRIVATE)
        
        // Load or prompt for drone name
        loadDroneName()
        
        // Setup drone name display
        setupDroneNameDisplay()
        
        // Initialize ViewModels
        basicAircraftControlVM = ViewModelProvider(this)[BasicAircraftControlVM::class.java]
        virtualStickVM = ViewModelProvider(this)[VirtualStickVM::class.java]
        
        // Initialize DroneController
        DroneController.init(basicAircraftControlVM, virtualStickVM)

        // Start listening for RC stick inputs (needed for manual override detection)
        virtualStickVM.listenRCStick()

        // Setup Manual Override checkbox
        setupManualOverrideCheckbox()

        // Setup AI Detection (AutoSensing) toggle & overlay
        setupAutoSensingToggle()
        setupEdgeDetectionToggle()
        updateDetectionTelemetryState()
        setupAircraftConnectionListener()
        setupVideoSourceState()
        setupMockVideoPreview()
        setupPhoneVideoPreview()
        setupMapExpandToggle()

        setupDetectedDroneProfileListener()
        updateWebRTCMetricsView(WebRTCStreamMetrics())
        updateEdgeMetricsView(lastEdgeMetrics)

        // Setup drone status indicator
        setupDroneStatusView()

        // Initialize LocationManager
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        startLocationUpdates()

        // Initialize Phone Sensors & Managers
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        
        // Acquire Multicast Lock to allow receiving UDP broadcasts
        multicastLock = wifiManager?.createMulticastLock("WildBridgeMulticastLock")
        multicastLock?.setReferenceCounted(true)
        multicastLock?.acquire()
        
        batteryManager = getSystemService(Context.BATTERY_SERVICE) as BatteryManager
        startSensorUpdates()
        
        // Get drone serial number
        fetchDroneSerialNumber()
        
        // Setup key listeners for telemetry
        setupKeyListeners()

        // Default field workflow: video mode, and SD card recording when available.
        scheduleDefaultCameraRecordingConfiguration()

        // Ensure MANAGE_EXTERNAL_STORAGE is granted so flight logs survive uninstalls.
        ensureManageExternalStoragePermission()

        // Sync any DJI TXT flight records accumulated since the last launch.
        syncDjiFlightLogsInBackground()

        // Start all servers
        startServers()
        
        // Show IP address
        showServerInfo()
    }
    
    // ==================== Mode Toggle (AUTO / MANUAL) ====================

    private fun setupManualOverrideCheckbox() {
        updateManualOverrideUI()

        findViewById<Switch>(R.id.cb_manual_override)?.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) DroneController.activateManualOverride()
            else DroneController.deactivateManualOverride()
            updateManualOverrideUI()
        }

        DroneController.manualOverrideListener = object : DroneController.ManualOverrideListener {
            override fun onManualOverrideActivated() {
                mainHandler.post { updateManualOverrideUI() }
            }
        }
    }

    private fun updateManualOverrideUI() {
        val isManual = DroneController.isManualOverrideActive
        // Blue = autonomous, Red = manual
        val color = if (isManual) 0xFFF44336.toInt() else 0xFF2196F3.toInt()
        val tint = ColorStateList.valueOf(color)
        findViewById<Switch>(R.id.cb_manual_override)?.let { sw ->
            sw.setOnCheckedChangeListener(null)
            sw.isChecked = isManual
            sw.text = if (isManual) "MANUAL" else "AUTO"
            sw.setTextColor(color)
            sw.trackTintList = tint
            sw.thumbTintList = ColorStateList.valueOf(if (isManual) 0xFFB71C1C.toInt() else 0xFF1565C0.toInt())
            sw.setOnCheckedChangeListener { _, isChecked ->
                if (isChecked) DroneController.activateManualOverride()
                else DroneController.deactivateManualOverride()
                updateManualOverrideUI()
            }
        }
    }

    // ==================== End Mode Toggle ====================

    private fun buildWebRTCOptions(): WebRTCMediaOptions {
        val preset = getWebRTCResolutionPreset()
        return if (preset == StreamResolutionPreset.AUTO) {
            WebRTCMediaOptions.native().copy(fps = getWebRTCFps())
        } else {
            WebRTCMediaOptions(
                videoResolutionWidth = preset.width,
                videoResolutionHeight = preset.height,
                fps = getWebRTCFps(),
                videoBitrate = preset.bitrate,
                videoCodec = "H264"
            )
        }
    }

    private fun getWebRTCFps(): Int {
        val storedFps = sharedPreferences.getInt(PREF_WEBRTC_FPS, DEFAULT_WEBRTC_FPS)
        return if (WEBRTC_FPS_OPTIONS.contains(storedFps)) storedFps else DEFAULT_WEBRTC_FPS
    }

    private fun getWebRTCResolutionPreset(): StreamResolutionPreset {
        return StreamResolutionPreset.fromPref(
            sharedPreferences.getString(PREF_WEBRTC_RESOLUTION, StreamResolutionPreset.AUTO.prefValue)
        )
    }

    private fun getVideoSourceMode(): VideoSourceMode {
        return VideoSourceMode.fromPref(sharedPreferences.getString(PREF_VIDEO_SOURCE, VideoSourceMode.DJI.prefValue))
    }

    private fun setupVideoSourceState() {
        if (!sharedPreferences.contains(PREF_VIDEO_SOURCE)) {
            val legacyMock = sharedPreferences.getBoolean(PREF_MOCK_VIDEO_ENABLED, false)
            sharedPreferences.edit()
                .putString(PREF_VIDEO_SOURCE, if (legacyMock) VideoSourceMode.MOCK.prefValue else VideoSourceMode.DJI.prefValue)
                .apply()
        }
        updateMockVideoVisibility()
        updatePhonePreviewVisibility()
        refreshMockTelemetryMode()
    }

    private fun setVideoSourceMode(mode: VideoSourceMode) {
        if (mode == VideoSourceMode.PHONE && !ensureCameraPermissionForPhoneSource(mode)) return
        sharedPreferences.edit()
            .putString(PREF_VIDEO_SOURCE, mode.prefValue)
            .putBoolean(PREF_MOCK_VIDEO_ENABLED, mode == VideoSourceMode.MOCK)
            .apply()
        webRTCStreamer?.setVideoSourceMode(mode)
        updateMockVideoVisibility()
        updatePhonePreviewVisibility()
        refreshMockTelemetryMode()
        invalidateOptionsMenu()
        val label = "Video source: ${mode.menuLabel}"
        Toast.makeText(this, label, Toast.LENGTH_SHORT).show()
        Log.i(TAG, label)
        if (activeDetectionSource() == DetectionSource.YOLO_ON_PHONE) {
            stopEdgeDetection()
            startEdgeDetection()
        }
    }

    private fun ensureCameraPermissionForPhoneSource(mode: VideoSourceMode): Boolean {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            return true
        }
        pendingVideoSourceAfterPermission = mode
        ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), REQUEST_PHONE_CAMERA_SOURCE)
        Toast.makeText(this, "Camera permission is needed for phone video source", Toast.LENGTH_SHORT).show()
        return false
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_PHONE_CAMERA_SOURCE) {
            val pendingMode = pendingVideoSourceAfterPermission
            pendingVideoSourceAfterPermission = null
            if (pendingMode == VideoSourceMode.PHONE && grantResults.firstOrNull() == PackageManager.PERMISSION_GRANTED) {
                setVideoSourceMode(VideoSourceMode.PHONE)
            } else {
                Toast.makeText(this, "Phone camera source unavailable without camera permission", Toast.LENGTH_SHORT).show()
            }
        }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (resultCode != RESULT_OK) return
        val uri = data?.data ?: return
        when (requestCode) {
            REQUEST_EDGE_MODEL_FILE -> storeEdgeModelSelection(uri)
            REQUEST_EDGE_LABELS_FILE -> storeEdgeFileSelection(uri, PREF_EDGE_LABELS_URI, PREF_EDGE_LABELS_NAME, "Edge labels")
        }
    }

    private fun storeEdgeModelSelection(uri: Uri) {
        val displayName = storeEdgeFileSelection(uri, PREF_EDGE_MODEL_URI, PREF_EDGE_MODEL_NAME, "Edge model")
        trySelectSiblingEdgeLabels(uri, displayName)
        if (activeDetectionSource() == DetectionSource.YOLO_ON_PHONE) {
            stopEdgeDetection()
            startEdgeDetection()
        }
    }

    private fun storeEdgeFileSelection(uri: Uri, uriPref: String, namePref: String, label: String): String {
        runCatching {
            contentResolver.takePersistableUriPermission(uri, Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }.onFailure { Log.d(TAG, "Could not persist $label URI permission: ${it.message}") }
        val displayName = uri.lastPathSegment?.substringAfterLast('/') ?: uri.toString().substringAfterLast('/')
        sharedPreferences.edit()
            .putString(uriPref, uri.toString())
            .putString(namePref, displayName)
            .apply()
        if (activeDetectionSource() == DetectionSource.YOLO_ON_PHONE) {
            stopEdgeDetection()
            startEdgeDetection()
        }
        Toast.makeText(this, "$label selected: $displayName", Toast.LENGTH_SHORT).show()
        invalidateOptionsMenu()
        return displayName
    }

    private fun setupMapExpandToggle() {
        val button = findViewById<ToggleButton>(R.id.button_map_expand) ?: return
        val expanded = sharedPreferences.getBoolean(PREF_MAP_EXPANDED, false)
        button.isChecked = expanded
        applyMapExpandedState(expanded)
        button.setOnCheckedChangeListener { _, isChecked ->
            sharedPreferences.edit().putBoolean(PREF_MAP_EXPANDED, isChecked).apply()
            applyMapExpandedState(isChecked)
        }
    }

    private fun applyMapExpandedState(expanded: Boolean) {
        val button = findViewById<ToggleButton>(R.id.button_map_expand)
        val compactWidth = resources.getDimensionPixelSize(R.dimen.uxsdk_150_dp)
        val compactHeight = resources.getDimensionPixelSize(R.dimen.uxsdk_100_dp)
        val width = if (expanded) {
            (resources.displayMetrics.widthPixels * 0.46f).toInt()
                .coerceAtLeast(dpToPx(320))
                .coerceAtMost(resources.displayMetrics.widthPixels - dpToPx(24))
        } else {
            compactWidth
        }
        val height = if (expanded) {
            (resources.displayMetrics.heightPixels * 0.44f).toInt()
                .coerceAtLeast(dpToPx(220))
                .coerceAtMost(resources.displayMetrics.heightPixels - dpToPx(96))
        } else {
            compactHeight
        }
        mapWidget.layoutParams = mapWidget.layoutParams.apply {
            this.width = width
            this.height = height
        }
        button?.contentDescription = if (expanded) "Minimize map" else "Expand map"
        mapWidget.requestLayout()
    }

    private fun dpToPx(value: Int): Int {
        return (value * resources.displayMetrics.density).toInt()
    }

    private fun actionRowAdapter(rows: List<SettingsActionRow>): ArrayAdapter<SettingsActionRow> {
        return object : ArrayAdapter<SettingsActionRow>(this, 0, rows) {
            override fun isEnabled(position: Int): Boolean = getItem(position)?.enabled == true

            override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
                val row = getItem(position) ?: SettingsActionRow("")
                val root = (convertView as? LinearLayout) ?: LinearLayout(context).apply {
                    orientation = LinearLayout.HORIZONTAL
                    gravity = android.view.Gravity.CENTER_VERTICAL
                    setPadding(dpToPx(18), dpToPx(12), dpToPx(14), dpToPx(12))
                    minimumHeight = dpToPx(68)
                }
                root.removeAllViews()
                root.alpha = if (row.enabled) 1.0f else 0.45f
                root.background = android.graphics.drawable.GradientDrawable().apply {
                    setColor(0xFFF7F9FC.toInt())
                    setStroke(dpToPx(1), 0xFFE1E7EF.toInt())
                }

                val textColumn = LinearLayout(context).apply {
                    orientation = LinearLayout.VERTICAL
                    layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f)
                }
                textColumn.addView(TextView(context).apply {
                    text = row.title
                    setTextColor(0xFF1F2937.toInt())
                    textSize = 15f
                    setTypeface(typeface, android.graphics.Typeface.BOLD)
                })
                row.detail?.takeIf { it.isNotBlank() }?.let { detail ->
                    textColumn.addView(TextView(context).apply {
                        text = detail
                        setTextColor(0xFF5F6F82.toInt())
                        textSize = 13f
                        maxLines = 1
                        ellipsize = android.text.TextUtils.TruncateAt.END
                    })
                }
                root.addView(textColumn)
                root.addView(TextView(context).apply {
                    text = "›"
                    setTextColor(0xFF78C7FF.toInt())
                    textSize = 24f
                    setPadding(dpToPx(12), 0, 0, 0)
                    visibility = if (row.enabled) View.VISIBLE else View.INVISIBLE
                })
                return root
            }
        }
    }

    private fun trySelectSiblingEdgeLabels(modelUri: Uri, modelName: String) {
        val labelsUri = findSiblingLabelsUri(modelUri, modelName) ?: return
        val labelsName = labelsUri.lastPathSegment?.substringAfterLast('/') ?: labelsUri.toString().substringAfterLast('/')
        if (readEdgeLabels(labelsUri).isEmpty()) return
        runCatching {
            contentResolver.takePersistableUriPermission(labelsUri, Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }.onFailure { Log.d(TAG, "Could not persist auto edge labels URI permission: ${it.message}") }
        sharedPreferences.edit()
            .putString(PREF_EDGE_LABELS_URI, labelsUri.toString())
            .putString(PREF_EDGE_LABELS_NAME, labelsName)
            .apply()
        Toast.makeText(this, "Edge labels auto-selected: $labelsName", Toast.LENGTH_SHORT).show()
    }

    private fun findSiblingLabelsUri(modelUri: Uri, modelName: String): Uri? {
        val folderId = if (DocumentsContract.isDocumentUri(this, modelUri)) {
            runCatching { DocumentsContract.getDocumentId(modelUri) }
                .getOrNull()
                ?.substringBeforeLast('/', missingDelimiterValue = "")
                ?.takeIf { it.isNotBlank() }
        } else {
            null
        }

        return folderId?.let { parentFolderId ->
            val candidateNames = candidateLabelNames(modelName)
            val siblingMatch = candidateNames
                .asSequence()
                .map { DocumentsContract.buildDocumentUri(modelUri.authority, "$parentFolderId/$it") }
                .firstOrNull { readEdgeLabels(it).isNotEmpty() }

            siblingMatch ?: run {
                val candidateNameSet = candidateNames.map { it.lowercase(java.util.Locale.US) }.toSet()
                val childrenUri = DocumentsContract.buildChildDocumentsUri(modelUri.authority, parentFolderId)
                runCatching {
                    contentResolver.query(
                        childrenUri,
                        arrayOf(DocumentsContract.Document.COLUMN_DOCUMENT_ID, DocumentsContract.Document.COLUMN_DISPLAY_NAME),
                        null,
                        null,
                        null
                    )?.use { cursor ->
                        val idIndex = cursor.getColumnIndexOrThrow(DocumentsContract.Document.COLUMN_DOCUMENT_ID)
                        val nameIndex = cursor.getColumnIndexOrThrow(DocumentsContract.Document.COLUMN_DISPLAY_NAME)
                        while (cursor.moveToNext()) {
                            val name = cursor.getString(nameIndex) ?: continue
                            if (candidateNameSet.contains(name.lowercase(java.util.Locale.US))) {
                                return@use DocumentsContract.buildDocumentUri(modelUri.authority, cursor.getString(idIndex))
                            }
                        }
                        null
                    }
                }.getOrElse { error ->
                    Log.d(TAG, "Could not scan model sibling labels: ${error.message}")
                    null
                }
            }
        }
    }

    private fun candidateLabelNames(modelName: String): List<String> {
        val base = modelName.substringBeforeLast('.')
        val simplified = base
            .removeSuffix("_dynamic_range_quant")
            .removeSuffix("_float32")
            .removeSuffix("_float16")
            .removeSuffix("_int8")
            .replace(Regex("_320$"), "")
        return listOf(
            "$base.txt",
            "${base}_labels.txt",
            "$simplified.txt",
            "${simplified}_labels.txt"
        )
    }

    private fun getEdgeModelUri(): Uri? {
        return sharedPreferences.getString(PREF_EDGE_MODEL_URI, null)?.let(Uri::parse)
    }

    private fun getEdgeLabels(): List<String> {
        val labelsUri = sharedPreferences.getString(PREF_EDGE_LABELS_URI, null)?.let(Uri::parse) ?: return listOf("person")
        return readEdgeLabels(labelsUri).ifEmpty { listOf("person") }
    }

    private fun getEdgeConfidenceThreshold(): Float {
        return sharedPreferences.getFloat(PREF_EDGE_CONFIDENCE_THRESHOLD, DEFAULT_EDGE_CONFIDENCE_THRESHOLD)
            .coerceIn(0.01f, 0.99f)
    }

    private fun readEdgeLabels(labelsUri: Uri): List<String> {
        return runCatching {
            contentResolver.openInputStream(labelsUri)?.bufferedReader()?.useLines { lines ->
                lines.map { it.trim() }.filter { it.isNotEmpty() }.toList()
            }.orEmpty()
        }.getOrElse { error ->
            Log.e(TAG, "Failed to read edge labels: ${error.message}", error)
            emptyList()
        }
    }

    private fun showEdgeFilePicker(requestCode: Int, title: String) {
        val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE)
            type = "*/*"
            putExtra(Intent.EXTRA_TITLE, title)
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION)
        }
        startActivityForResult(intent, requestCode)
    }

    private fun isMockVideoEnabled(): Boolean {
        return getVideoSourceMode() == VideoSourceMode.MOCK
    }

    private fun shouldUseMockTelemetry(): Boolean {
        return getVideoSourceMode() == VideoSourceMode.MOCK || getVideoSourceMode() == VideoSourceMode.PHONE
    }

    private fun setupAircraftConnectionListener() {
        val initialConnectionState = flightControllerConnectionKey.get(true)
        applyAircraftConnectionState(initialConnectionState, forceDroneSourceDefault = initialConnectionState)
        KeyManager.getInstance().listen(flightControllerConnectionKey, this) { _, newValue ->
            mainHandler.post {
                applyAircraftConnectionState(newValue == true)
            }
        }
    }

    private fun applyAircraftConnectionState(isConnected: Boolean, forceDroneSourceDefault: Boolean = false) {
        val wasConnected = aircraftConnected
        aircraftConnected = isConnected
        if (shouldSwitchToDroneVideoSource(isConnected, wasConnected, forceDroneSourceDefault)) {
            sharedPreferences.edit()
                .putString(PREF_VIDEO_SOURCE, VideoSourceMode.DJI.prefValue)
                .putBoolean(PREF_MOCK_VIDEO_ENABLED, false)
                .apply()
            webRTCStreamer?.setVideoSourceMode(VideoSourceMode.DJI)
        }
        if (!isConnected && isDetectionsEnabled() && getDetectionSource() == DetectionSource.DJI_ONBOARD) {
            setDetectionsEnabled(false)
        }
        updateMockVideoVisibility()
        updatePhonePreviewVisibility()
        refreshMockTelemetryMode()
        invalidateOptionsMenu()
    }

    private fun shouldSwitchToDroneVideoSource(
        isConnected: Boolean,
        wasConnected: Boolean,
        forceDroneSourceDefault: Boolean
    ): Boolean {
        val shouldSelectDroneSource = forceDroneSourceDefault || !wasConnected
        return isConnected && shouldSelectDroneSource && getVideoSourceMode() != VideoSourceMode.DJI
    }

    private fun updateMockVideoVisibility() {
        findViewById<Switch>(R.id.sw_mock_video)?.let { switch ->
            switch.visibility = android.view.View.GONE
            switch.isChecked = isMockVideoEnabled()
            updateMockVideoToggleUi(switch.isChecked)
        }
        updateMockPreviewVisibility()
    }

    private fun setupPhoneVideoPreview() {
        findViewById<TextureView>(R.id.phone_camera_preview)?.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
                updatePhonePreviewVisibility()
            }

            override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) = Unit

            override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
                stopPhoneCameraPreview()
                return true
            }

            override fun onSurfaceTextureUpdated(surface: SurfaceTexture) = Unit
        }
        updatePhonePreviewVisibility()
    }

    private fun updatePhonePreviewVisibility() {
        val preview = findViewById<TextureView>(R.id.phone_camera_preview)
        val shouldShow = getVideoSourceMode() == VideoSourceMode.PHONE
        preview?.visibility = if (shouldShow) android.view.View.VISIBLE else android.view.View.GONE
        findViewById<TextView>(R.id.phone_camera_preview_label)?.visibility = if (shouldShow) android.view.View.VISIBLE else android.view.View.GONE
        if (shouldShow && preview?.isAvailable == true) {
            detectionOverlay?.setVideoScaleMode(DetectionOverlayView.VideoScaleMode.CENTER_CROP)
            startPhoneCameraPreview(preview.surfaceTexture ?: return)
        } else if (!shouldShow) {
            stopPhoneCameraPreview()
        }
    }

    private fun configurePhonePreviewTransform(preview: TextureView, sourceWidth: Int, sourceHeight: Int) {
        val viewWidth = preview.width.toFloat().takeIf { it > 0f } ?: return
        val viewHeight = preview.height.toFloat().takeIf { it > 0f } ?: return
        val rotation = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) display?.rotation else windowManager.defaultDisplay.rotation
        val matrix = Matrix()
        val viewRect = RectF(0f, 0f, viewWidth, viewHeight)
        val centerX = viewRect.centerX()
        val centerY = viewRect.centerY()
        if (rotation == Surface.ROTATION_90 || rotation == Surface.ROTATION_270) {
            val bufferRect = RectF(0f, 0f, sourceHeight.toFloat(), sourceWidth.toFloat()).apply {
                offset(centerX - centerX(), centerY - centerY())
            }
            matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL)
            val scale = maxOf(viewHeight / sourceHeight.toFloat(), viewWidth / sourceWidth.toFloat())
            matrix.postScale(scale, scale, centerX, centerY)
            matrix.postRotate(90f * (rotation - 2), centerX, centerY)
        } else {
            val scale = maxOf(viewWidth / sourceWidth.toFloat(), viewHeight / sourceHeight.toFloat())
            matrix.postScale(scale, scale, centerX, centerY)
        }
        preview.setTransform(matrix)
    }

    private fun startPhoneCameraPreview(surfaceTexture: SurfaceTexture) {
        val canStart = getVideoSourceMode() == VideoSourceMode.PHONE &&
            ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED &&
            phoneCameraDevice == null
        if (!canStart) return

        runCatching {
            val cameraManager = getSystemService(Context.CAMERA_SERVICE) as CameraManager
            val cameraId = cameraManager.cameraIdList.firstOrNull { id ->
                cameraManager.getCameraCharacteristics(id)
                    .get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK
            } ?: cameraManager.cameraIdList.firstOrNull()

            if (cameraId == null) {
                Log.e(TAG, "No phone camera available for preview")
                stopPhoneCameraPreview()
            } else {
                val characteristics = cameraManager.getCameraCharacteristics(cameraId)
                val previewSize = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                    ?.getOutputSizes(SurfaceTexture::class.java)
                    ?.sortedWith(compareBy({ kotlin.math.abs(it.width - 1920) + kotlin.math.abs(it.height - 1080) }, { it.width * it.height }))
                    ?.firstOrNull()
                val phoneFrameSize = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                    ?.getOutputSizes(ImageFormat.YUV_420_888)
                    ?.sortedWith(compareBy({ kotlin.math.abs(it.width - 1280) + kotlin.math.abs(it.height - 720) }, { it.width * it.height }))
                    ?.firstOrNull()

                val width = previewSize?.width ?: 1920
                val height = previewSize?.height ?: 1080
                surfaceTexture.setDefaultBufferSize(width, height)
                val surface = Surface(surfaceTexture)
                phonePreviewSurface = surface
                findViewById<TextureView>(R.id.phone_camera_preview)?.let {
                    configurePhonePreviewTransform(it, width, height)
                }

                val thread = HandlerThread("WildBridgePhonePreview").also { it.start() }
                phoneCameraThread = thread
                phoneCameraHandler = Handler(thread.looper)
                val frameWidth = phoneFrameSize?.width ?: 1280
                val frameHeight = phoneFrameSize?.height ?: 720
                detectionOverlay?.setSourceFrameSize(frameWidth, frameHeight)
                phoneImageReader = ImageReader.newInstance(frameWidth, frameHeight, ImageFormat.YUV_420_888, 3).apply {
                    setOnImageAvailableListener({ reader -> handlePhoneInferenceImage(reader) }, phoneCameraHandler)
                }
                Log.i(TAG, "Phone shared frame reader configured: ${frameWidth}x${frameHeight}")

                cameraManager.openCamera(cameraId, object : CameraDevice.StateCallback() {
                    override fun onOpened(camera: CameraDevice) {
                        phoneCameraDevice = camera
                        createPhonePreviewSession(camera, surface)
                        Log.i(TAG, "Phone camera preview opened: $cameraId ${width}x${height}")
                    }

                    override fun onDisconnected(camera: CameraDevice) {
                        Log.w(TAG, "Phone camera preview disconnected")
                        stopPhoneCameraPreview()
                    }

                    override fun onError(camera: CameraDevice, error: Int) {
                        Log.e(TAG, "Phone camera preview error: $error")
                        stopPhoneCameraPreview()
                    }
                }, phoneCameraHandler)
            }
        }.onFailure { error ->
            Log.e(TAG, "Failed to start phone camera preview: ${error.message}", error)
            stopPhoneCameraPreview()
        }
    }

    private fun createPhonePreviewSession(camera: CameraDevice, surface: Surface) {
        try {
            val request = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                addTarget(surface)
                phoneImageReader?.surface?.let { addTarget(it) }
                set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO)
            }
            val surfaces = listOfNotNull(surface, phoneImageReader?.surface)
            camera.createCaptureSession(surfaces, object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(session: CameraCaptureSession) {
                    phoneCameraSession = session
                    runCatching { session.setRepeatingRequest(request.build(), null, phoneCameraHandler) }
                        .onFailure { Log.e(TAG, "Failed to start phone preview repeating request: ${it.message}", it) }
                }

                override fun onConfigureFailed(session: CameraCaptureSession) {
                    Log.e(TAG, "Phone camera preview session configure failed")
                    stopPhoneCameraPreview()
                }
            }, phoneCameraHandler)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to create phone camera preview session: ${e.message}", e)
            stopPhoneCameraPreview()
        }
    }

    private fun stopPhoneCameraPreview() {
        runCatching { phoneCameraSession?.stopRepeating() }
        runCatching { phoneCameraSession?.close() }
        phoneCameraSession = null
        runCatching { phoneCameraDevice?.close() }
        phoneCameraDevice = null
        runCatching { phoneImageReader?.close() }
        phoneImageReader = null
        runCatching { phonePreviewSurface?.release() }
        phonePreviewSurface = null
        phoneCameraThread?.quitSafely()
        phoneCameraThread = null
        phoneCameraHandler = null
        phoneInferenceBusy.set(false)
        lastPhoneEdgeFrameNs = 0L
    }

    private fun handlePhoneInferenceImage(reader: ImageReader) {
        val image = reader.acquireLatestImage() ?: return
        val timestampNs = System.nanoTime()
        val isPhoneSource = getVideoSourceMode() == VideoSourceMode.PHONE
        if (!isPhoneSource) {
            image.close()
        } else {
            SharedPhoneCameraFrameSource.offerImage(image, timestampNs)
            val controller = edgeDetectionController
            val canRunInference = controller != null &&
                timestampNs - lastPhoneEdgeFrameNs >= PHONE_EDGE_FRAME_INTERVAL_NS &&
                phoneInferenceBusy.compareAndSet(false, true)
            if (!canRunInference) {
                image.close()
            } else {
                lastPhoneEdgeFrameNs = timestampNs
                controller.onYuv420Image(image, timestampNs) {
                    phoneInferenceBusy.set(false)
                }
            }
        }
    }

    private fun setupMockVideoPreview() {
        findViewById<TextureView>(R.id.mock_video_preview)?.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
                updateMockPreviewVisibility()
            }

            override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) = Unit

            override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
                stopMockVideoPreview()
                return true
            }

            override fun onSurfaceTextureUpdated(surface: SurfaceTexture) = Unit
        }
        updateMockPreviewVisibility()
    }

    private fun updateMockPreviewVisibility() {
        val preview = findViewById<TextureView>(R.id.mock_video_preview)
        val label = findViewById<TextView>(R.id.mock_video_preview_label)
        val shouldShow = isMockVideoEnabled()
        preview?.visibility = if (shouldShow) android.view.View.VISIBLE else android.view.View.GONE
        label?.visibility = if (shouldShow) android.view.View.VISIBLE else android.view.View.GONE
        if (shouldShow && preview?.isAvailable == true) {
            startMockVideoPreview(preview.surfaceTexture ?: return)
        } else if (!shouldShow) {
            stopMockVideoPreview()
        }
    }

    private fun startMockVideoPreview(surfaceTexture: SurfaceTexture) {
        if (!isMockVideoEnabled()) return
        if (mockPreviewPlayer != null) {
            runCatching { mockPreviewPlayer?.start() }
            return
        }

        try {
            val descriptor = assets.openFd("mock_video/jellyfish_1080_10s_5mb.mp4")
            val surface = Surface(surfaceTexture)
            mockPreviewPlayer = MediaPlayer().apply {
                setDataSource(descriptor.fileDescriptor, descriptor.startOffset, descriptor.length)
                setSurface(surface)
                isLooping = true
                setOnPreparedListener { player -> player.start() }
                setOnErrorListener { _, what, extra ->
                    Log.e(TAG, "Mock preview player error: what=$what extra=$extra")
                    true
                }
                prepareAsync()
            }
            descriptor.close()
            surface.release()
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start mock preview: ${e.message}", e)
            stopMockVideoPreview()
        }
    }

    private fun stopMockVideoPreview() {
        mockPreviewPlayer?.let { player ->
            runCatching {
                player.stop()
            }
            runCatching {
                player.reset()
                player.release()
            }
        }
        mockPreviewPlayer = null
    }

    private fun updateMockVideoToggleUi(isEnabled: Boolean) {
        findViewById<Switch>(R.id.sw_mock_video)?.let { switch ->
            switch.text = if (isEnabled) "MOCK VIDEO" else "DJI VIDEO"
            switch.setTextColor(if (isEnabled) 0xFFFFD166.toInt() else 0xFFDDDDDD.toInt())
        }
    }

    private fun refreshMockTelemetryMode() {
        TelemetryProvider.configureMockTelemetry(
            enabled = shouldUseMockTelemetry(),
            baseLatitude = phoneLocation?.latitude,
            baseLongitude = phoneLocation?.longitude,
            baseAltitude = phoneLocation?.altitude
        )
        cachedTelemetryJson = buildTelemetryJson()
    }

    private fun setupDetectedDroneProfileListener() {
        applyDetectedDroneProfile(productTypeKey.get(ProductType.UNKNOWN) ?: ProductType.UNKNOWN)
        KeyManager.getInstance().listen(productTypeKey, this) { _, newValue ->
            mainHandler.post {
                applyDetectedDroneProfile(newValue ?: ProductType.UNKNOWN)
            }
        }
    }

    private fun applyDetectedDroneProfile(productType: ProductType) {
        val controlProfile = DroneControlProfiles.fromProductType(productType)
        val controlLabel = when (controlProfile) {
            DroneControlProfile.MATRICE_350_RTK -> "CTRL M350"
            DroneControlProfile.MINI_4_PRO -> "CTRL MINI4"
            DroneControlProfile.MAVIC_3_ENTERPRISE -> "CTRL MAVIC3"
        }
        findViewById<TextView>(R.id.text_control_profile)?.text = controlLabel
        Log.i(TAG, "Detected product $productType -> using ${controlProfile.displayName} profile")
    }

    private fun updateWebRTCMetricsView(metrics: WebRTCStreamMetrics) {
        lastWebRTCMetrics = metrics
        if (getVideoSourceMode() == VideoSourceMode.DJI) {
            detectionOverlay?.setVideoScaleMode(DetectionOverlayView.VideoScaleMode.CENTER_INSIDE)
        }
        if (metrics.sourceWidth > 0 && metrics.sourceHeight > 0) {
            detectionOverlay?.setSourceFrameSize(metrics.sourceWidth, metrics.sourceHeight)
        }
        findViewById<TextView>(R.id.text_webrtc_metrics)?.text = metrics.compactLabel()
    }

    private fun WebRTCStreamMetrics.toTelemetryJson(): String {
        fun escapeJson(value: String): String = value.replace("\\", "\\\\").replace("\"", "\\\"")
        val lastErrorJson = lastError?.let { "\"${escapeJson(it)}\"" } ?: "null"
        return """{"sourceWidth":$sourceWidth,"sourceHeight":$sourceHeight,"outputWidth":$outputWidth,"outputHeight":$outputHeight,"requestedWidth":$requestedWidth,"requestedHeight":$requestedHeight,"targetFps":$targetFps,"inputFps":$inputFps,"outputFps":$outputFps,"droppedFps":$droppedFps,"averageFrameProcessingMs":$averageFrameProcessingMs,"totalFrames":$totalFrames,"totalDroppedFrames":$totalDroppedFrames,"processingErrors":$processingErrors,"observerCount":$observerCount,"activeCamera":"${escapeJson(activeCamera)}","status":"${escapeJson(status)}","configuredFps":$configuredFps,"saturationState":"${escapeJson(saturationState)}","scaleMode":"${escapeJson(scaleMode)}","recoveryCount":$recoveryCount,"lastError":$lastErrorJson}"""
    }

    /**
     * Start WHIP publishing on the existing WebRTC streamer.
     * Called automatically when the bridge connects to the telemetry server.
     */
    private fun startWhipPublishing(whipUrl: String) {
        lastWhipUrl = whipUrl  // Remember for FPS/Quality mode restarts
        val streamer = webRTCStreamer
        if (streamer == null) {
            Log.w(TAG, "Cannot start WHIP — WebRTCStreamer not initialized yet")
            return
        }
        try {
            streamer.startWhip(whipUrl)
            Log.i(TAG, "WHIP publishing started: $whipUrl")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start WHIP publishing: ${e.message}", e)
        }
    }

    // ==================== End Video Mode Toggle ====================

    // ==================== AutoSensing (AI Detection) Toggle ====================

    private fun setupAutoSensingToggle() {
        detectionOverlay = findViewById(R.id.detection_overlay)

        val sw = findViewById<Switch>(R.id.sw_auto_sensing) ?: return
        sw.setOnCheckedChangeListener(null)
        sw.isChecked = isDetectionsEnabled() && getDetectionSource() == DetectionSource.DJI_ONBOARD
        sw.visibility = android.view.View.GONE
    }

    private fun isDetectionsEnabled(): Boolean {
        return sharedPreferences.getBoolean(
            PREF_DETECTIONS_ENABLED,
            sharedPreferences.getString(PREF_DETECTION_SOURCE, null) != null && getDetectionSource() != DetectionSource.NONE
        )
    }

    private fun activeDetectionSource(): DetectionSource {
        return if (isDetectionsEnabled()) getDetectionSource() else DetectionSource.NONE
    }

    private fun isDetectionActiveForUi(): Boolean {
        return when (activeDetectionSource()) {
            DetectionSource.NONE -> false
            DetectionSource.DJI_ONBOARD -> isAutoSensingActive
            DetectionSource.YOLO_ON_PHONE -> edgeDetectionController != null
        }
    }

    private fun detectionMenuLabel(): String {
        return if (isDetectionActiveForUi()) {
            "Detections On (${getDetectionSource().menuLabel})"
        } else {
            "Detections Off"
        }
    }

    private fun getDetectionSource(): DetectionSource {
        val stored = sharedPreferences.getString(PREF_DETECTION_SOURCE, null)
        if (stored == null && sharedPreferences.getBoolean(PREF_EDGE_DETECTION_ENABLED, false)) {
            return DetectionSource.YOLO_ON_PHONE
        }
        return DetectionSource.fromPref(stored).takeIf { it != DetectionSource.NONE } ?: DetectionSource.YOLO_ON_PHONE
    }

    private fun setDetectionSource(source: DetectionSource) {
        val selectedSource = source.takeIf { it != DetectionSource.NONE } ?: DetectionSource.YOLO_ON_PHONE
        if (selectedSource == DetectionSource.DJI_ONBOARD && !aircraftConnected) {
            Toast.makeText(this, "DJI onboard detections need a connected drone", Toast.LENGTH_SHORT).show()
            return
        }

        stopAutoSensing()
        stopEdgeDetection()

        sharedPreferences.edit()
            .putString(PREF_DETECTION_SOURCE, selectedSource.prefValue)
            .putBoolean(PREF_EDGE_DETECTION_ENABLED, isDetectionsEnabled() && selectedSource == DetectionSource.YOLO_ON_PHONE)
            .apply()

        findViewById<Switch>(R.id.sw_auto_sensing)?.isChecked = isDetectionsEnabled() && selectedSource == DetectionSource.DJI_ONBOARD
        findViewById<Switch>(R.id.sw_edge_detection)?.isChecked = isDetectionsEnabled() && selectedSource == DetectionSource.YOLO_ON_PHONE

        when (activeDetectionSource()) {
            DetectionSource.NONE -> updateEdgeMetricsView(EdgeDetectionMetrics(status = "off"))
            DetectionSource.DJI_ONBOARD -> startAutoSensing()
            DetectionSource.YOLO_ON_PHONE -> startEdgeDetection()
        }

        updateDetectionTelemetryState()
        rebuildTelemetryCache()
        invalidateOptionsMenu()
    }

    private fun setDetectionsEnabled(enabled: Boolean) {
        if (enabled && getDetectionSource() == DetectionSource.DJI_ONBOARD && !aircraftConnected) {
            Toast.makeText(this, "DJI onboard detections need a connected drone", Toast.LENGTH_SHORT).show()
            return
        }

        stopAutoSensing()
        stopEdgeDetection()

        sharedPreferences.edit()
            .putBoolean(PREF_DETECTIONS_ENABLED, enabled)
            .putBoolean(PREF_EDGE_DETECTION_ENABLED, enabled && getDetectionSource() == DetectionSource.YOLO_ON_PHONE)
            .apply()

        findViewById<Switch>(R.id.sw_auto_sensing)?.isChecked = enabled && getDetectionSource() == DetectionSource.DJI_ONBOARD
        findViewById<Switch>(R.id.sw_edge_detection)?.isChecked = enabled && getDetectionSource() == DetectionSource.YOLO_ON_PHONE

        when (activeDetectionSource()) {
            DetectionSource.NONE -> {
                updateEdgeMetricsView(EdgeDetectionMetrics(status = "off"))
                Toast.makeText(this, "Detections disabled", Toast.LENGTH_SHORT).show()
            }
            DetectionSource.DJI_ONBOARD -> startAutoSensing()
            DetectionSource.YOLO_ON_PHONE -> startEdgeDetection()
        }

        updateDetectionTelemetryState()
        rebuildTelemetryCache()
        invalidateOptionsMenu()
    }

    private fun updateDetectionTelemetryState() {
        val source = activeDetectionSource()
        TelemetryProvider.currentDetectionSource = source.prefValue
        TelemetryProvider.currentDetectionActive = when (source) {
            DetectionSource.NONE -> false
            DetectionSource.DJI_ONBOARD -> isAutoSensingActive
            DetectionSource.YOLO_ON_PHONE -> edgeDetectionController != null
        }
        TelemetryProvider.currentDetectionModel = when (source) {
            DetectionSource.YOLO_ON_PHONE -> sharedPreferences.getString(PREF_EDGE_MODEL_NAME, null)
            else -> null
        }
        TelemetryProvider.currentDetectionThreshold = when (source) {
            DetectionSource.YOLO_ON_PHONE -> getEdgeConfidenceThreshold()
            else -> null
        }
    }

    private fun showDetectionSourceDialog() {
        val allSources = arrayOf(DetectionSource.DJI_ONBOARD, DetectionSource.YOLO_ON_PHONE)
        val labels = allSources.map { source ->
            if (source == DetectionSource.DJI_ONBOARD && !aircraftConnected) {
                "${source.menuLabel} (connect drone)"
            } else {
                source.menuLabel
            }
        }.toTypedArray()
        val checkedIndex = allSources.indexOf(getDetectionSource()).coerceAtLeast(0)

        AlertDialog.Builder(this)
            .setTitle("Detection source")
            .setSingleChoiceItems(labels, checkedIndex) { dialog, which ->
                setDetectionSource(allSources[which])
                dialog.dismiss()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun showDetectionSettingsDialog() {
        val modelName = sharedPreferences.getString(PREF_EDGE_MODEL_NAME, "Select...")
        val labelsName = sharedPreferences.getString(PREF_EDGE_LABELS_NAME, "Default person")
        val confidence = (getEdgeConfidenceThreshold() * 100).toInt()
        val rows = listOf(
            SettingsActionRow("Source", getDetectionSource().menuLabel),
            SettingsActionRow("YOLO model", modelName),
            SettingsActionRow("YOLO labels", labelsName),
            SettingsActionRow("YOLO confidence", "$confidence%")
        )

        AlertDialog.Builder(this)
            .setTitle("Detection Settings")
            .setAdapter(actionRowAdapter(rows)) { dialog, which ->
                dialog.dismiss()
                when (which) {
                    0 -> showDetectionSourceDialog()
                    1 -> showEdgeFilePicker(REQUEST_EDGE_MODEL_FILE, "Select YOLO TFLite model")
                    2 -> showEdgeFilePicker(REQUEST_EDGE_LABELS_FILE, "Select model labels")
                    3 -> showEdgeConfidenceDialog()
                }
            }
            .setNegativeButton("Close", null)
            .show()
    }

    private fun applyDetectedTargets(targets: List<DetectedTarget>) {
        currentDetectedTargets = targets
        TelemetryProvider.currentDetectedTargets = targets
        updateDetectionTelemetryState()
        rebuildTelemetryCache()
        mainHandler.post { detectionOverlay?.setTargets(targets) }
    }

    private fun startAutoSensing() {
        if (isAutoSensingActive) return
        try {
            val manager = IntelligentFlightManager.getInstance()
            if (!isAutoSensingListenerRegistered) {
                manager.addAutoSensingInfoListener(autoSensingInfoListener)
                isAutoSensingListenerRegistered = true
            }
            manager.startAutoSensing(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    isAutoSensingActive = true
                    updateDetectionTelemetryState()
                    rebuildTelemetryCache()
                    Log.i(TAG, "AutoSensing started")
                }
                override fun onFailure(error: IDJIError) {
                    isAutoSensingActive = false
                    removeAutoSensingListener()
                    updateDetectionTelemetryState()
                    rebuildTelemetryCache()
                    Log.e(TAG, "AutoSensing start failed: ${error.description()}")
                }
            })
        } catch (e: Exception) {
            updateDetectionTelemetryState()
            rebuildTelemetryCache()
            Log.e(TAG, "AutoSensing start exception: ${e.message}")
        }
    }

    private fun stopAutoSensing() {
        clearAutoSensingState()
        if (!isAutoSensingActive) {
            removeAutoSensingListener()
            return
        }
        try {
            IntelligentFlightManager.getInstance().stopAutoSensing(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    Log.i(TAG, "AutoSensing stopped")
                }
                override fun onFailure(error: IDJIError) {
                    Log.e(TAG, "AutoSensing stop failed: ${error.description()}")
                }
            })
        } catch (e: Exception) {
            Log.e(TAG, "AutoSensing stop exception: ${e.message}")
        } finally {
            isAutoSensingActive = false
            removeAutoSensingListener()
            updateDetectionTelemetryState()
            rebuildTelemetryCache()
        }
    }

    private fun removeAutoSensingListener() {
        if (!isAutoSensingListenerRegistered) return
        try {
            IntelligentFlightManager.getInstance().removeAutoSensingInfoListener(autoSensingInfoListener)
        } catch (e: Exception) {
            Log.e(TAG, "AutoSensing listener removal exception: ${e.message}")
        } finally {
            isAutoSensingListenerRegistered = false
        }
    }

    private fun clearAutoSensingState() {
        currentDetectedTargets = emptyList()
        TelemetryProvider.currentDetectedTargets = emptyList()
        updateDetectionTelemetryState()
        rebuildTelemetryCache()
        mainHandler.post { detectionOverlay?.clearTargets() }
    }

    // ==================== End AutoSensing Toggle ====================

    // ==================== Edge Detection Toggle ====================

    private fun setupEdgeDetectionToggle() {
        val sw = findViewById<Switch>(R.id.sw_edge_detection) ?: return
        sw.setOnCheckedChangeListener(null)
        sw.isChecked = isDetectionsEnabled() && getDetectionSource() == DetectionSource.YOLO_ON_PHONE
        sw.visibility = android.view.View.GONE
        updateEdgeDetectionToggleUi(isDetectionsEnabled() && getDetectionSource() == DetectionSource.YOLO_ON_PHONE)
    }

    private fun isEdgeDetectionEnabled(): Boolean {
        return isDetectionsEnabled() && getDetectionSource() == DetectionSource.YOLO_ON_PHONE
    }

    private sealed interface EdgeDetectionStartCheck {
        data class Ready(val sourceMode: VideoSourceMode, val modelUri: Uri) : EdgeDetectionStartCheck
        data class UnsupportedSource(val sourceMode: VideoSourceMode) : EdgeDetectionStartCheck
        data class MissingModel(val sourceMode: VideoSourceMode) : EdgeDetectionStartCheck
        object WaitingForDjiVideo : EdgeDetectionStartCheck
    }

    private fun startEdgeDetection() {
        if (edgeDetectionController != null) return

        val startCheck = edgeDetectionStartCheck(getVideoSourceMode(), getEdgeModelUri(), webRTCStreamer)
        if (startCheck !is EdgeDetectionStartCheck.Ready) {
            handleEdgeDetectionStartFailure(startCheck)
            return
        }

        clearAutoSensingState()

        val controller = createEdgeDetectionController(startCheck)
        edgeDetectionController = controller

        configureDetectionOverlay(startCheck.sourceMode)
        controller.start()
        updateDetectionTelemetryState()
        rebuildTelemetryCache()

        attachEdgeDetectionSource(startCheck.sourceMode, controller)
        showEdgeDetectionEnabledMessage()
    }

    private fun edgeDetectionStartCheck(
        sourceMode: VideoSourceMode,
        modelUri: Uri?,
        streamer: WebRTCStreamer?
    ): EdgeDetectionStartCheck {
        return when {
            sourceMode == VideoSourceMode.MOCK -> {
                EdgeDetectionStartCheck.UnsupportedSource(sourceMode)
            }
            modelUri == null -> {
                EdgeDetectionStartCheck.MissingModel(sourceMode)
            }
            sourceMode == VideoSourceMode.DJI && streamer == null -> {
                EdgeDetectionStartCheck.WaitingForDjiVideo
            }
            else -> {
                EdgeDetectionStartCheck.Ready(sourceMode = sourceMode, modelUri = modelUri)
            }
        }
    }

    private fun handleEdgeDetectionStartFailure(startCheck: EdgeDetectionStartCheck) {
        when (startCheck) {
            is EdgeDetectionStartCheck.UnsupportedSource -> {
                setDetectionsEnabled(false)
                updateEdgeMetricsView(
                    EdgeDetectionMetrics(status = "source", source = startCheck.sourceMode.prefValue)
                )
                Toast.makeText(
                    this,
                    "Edge detection supports drone and phone camera sources",
                    Toast.LENGTH_SHORT
                ).show()
            }
            is EdgeDetectionStartCheck.MissingModel -> {
                setDetectionsEnabled(false)
                updateEdgeMetricsView(
                    EdgeDetectionMetrics(status = "no-model", source = startCheck.sourceMode.prefValue)
                )
                showEdgeFilePicker(REQUEST_EDGE_MODEL_FILE, "Select YOLO TFLite model")
                Toast.makeText(this, "Select a YOLO .tflite model first", Toast.LENGTH_SHORT).show()
            }
            EdgeDetectionStartCheck.WaitingForDjiVideo -> {
                Toast.makeText(this, "Edge detector will be ready after video starts", Toast.LENGTH_SHORT).show()
            }
            is EdgeDetectionStartCheck.Ready -> Unit
        }
    }

    private fun createEdgeDetectionController(
        startCheck: EdgeDetectionStartCheck.Ready
    ): EdgeDetectionController {
        return EdgeDetectionController(
            context = applicationContext,
            config = EdgeDetectionConfig(
                modelUri = startCheck.modelUri,
                labels = getEdgeLabels(),
                sourceLabel = startCheck.sourceMode.prefValue,
                confidenceThreshold = getEdgeConfidenceThreshold()
            ),
            onTargets = { targets ->
                if (activeDetectionSource() == DetectionSource.YOLO_ON_PHONE) {
                    applyDetectedTargets(targets)
                }
            },
            onMetrics = { metrics ->
                lastEdgeMetrics = metrics
                mainHandler.post { updateEdgeMetricsView(metrics) }
            }
        )
    }

    private fun configureDetectionOverlay(sourceMode: VideoSourceMode) {
        when (sourceMode) {
            VideoSourceMode.DJI -> {
                detectionOverlay?.setVideoScaleMode(DetectionOverlayView.VideoScaleMode.CENTER_INSIDE)
                detectionOverlay?.setSourceFrameSize(
                    lastWebRTCMetrics.sourceWidth.takeIf { it > 0 } ?: 16,
                    lastWebRTCMetrics.sourceHeight.takeIf { it > 0 } ?: 9
                )
            }
            VideoSourceMode.PHONE -> {
                detectionOverlay?.setVideoScaleMode(DetectionOverlayView.VideoScaleMode.CENTER_CROP)
            }
            VideoSourceMode.MOCK -> Unit
        }
    }

    private fun attachEdgeDetectionSource(
        sourceMode: VideoSourceMode,
        controller: EdgeDetectionController
    ) {
        if (sourceMode == VideoSourceMode.DJI) {
            webRTCStreamer?.setEdgeDetectionFrameListener(controller)
            return
        }
        webRTCStreamer?.setEdgeDetectionFrameListener(null)
        stopPhoneCameraPreview()
        updatePhonePreviewVisibility()
    }

    private fun showEdgeDetectionEnabledMessage() {
        Toast.makeText(this, "Edge detection enabled", Toast.LENGTH_SHORT).show()
        Log.i(TAG, "Edge detection enabled")
    }

    private fun stopEdgeDetection() {
        val controller = edgeDetectionController ?: return
        webRTCStreamer?.setEdgeDetectionFrameListener(null)
        controller.dispose()
        edgeDetectionController = null
        clearAutoSensingState()
        phoneInferenceBusy.set(false)
        if (getVideoSourceMode() == VideoSourceMode.PHONE) {
            stopPhoneCameraPreview()
            updatePhonePreviewVisibility()
        }
        updateDetectionTelemetryState()
        rebuildTelemetryCache()
        updateEdgeMetricsView(EdgeDetectionMetrics(status = "off"))
        Toast.makeText(this, "Edge detection disabled", Toast.LENGTH_SHORT).show()
        Log.i(TAG, "Edge detection disabled")
    }

    private fun updateEdgeDetectionToggleUi(isEnabled: Boolean) {
        findViewById<Switch>(R.id.sw_edge_detection)?.let { switch ->
            switch.text = if (isEnabled) "EDGE DETECT" else "EDGE OFF"
            switch.setTextColor(if (isEnabled) 0xFFFFD166.toInt() else 0xFFDDDDDD.toInt())
        }
    }

    private fun updateEdgeMetricsView(metrics: EdgeDetectionMetrics) {
        lastEdgeMetrics = metrics
        findViewById<TextView>(R.id.text_edge_metrics)?.text = metrics.compactLabel()
    }

    // ==================== End Edge Detection Toggle ====================

    // ==================== Drone Status View ====================

    private fun setupDroneStatusView() {
        DroneController.droneStatusListener = object : DroneController.DroneStatusListener {
            override fun onDroneStatusChanged(status: DroneController.DroneStatus) {
                WildBridgeFlightLogger.logStatus(status.name)
                mainHandler.post { updateDroneStatusView(status) }
            }
        }
        updateDroneStatusView(DroneController.droneStatus)
    }

    private fun updateDroneStatusView(appStatus: DroneController.DroneStatus) {
        val statusTv = findViewById<TextView>(R.id.text_drone_status) ?: return
        // Upgrade IDLE → HOVERING when the FC says the drone is airborne
        val resolved = if (appStatus == DroneController.DroneStatus.IDLE && isFlyingKey.get(false)) {
            DroneController.DroneStatus.HOVERING
        } else {
            appStatus
        }
        val (label, color) = when (resolved) {
            DroneController.DroneStatus.IDLE            -> Pair("IDLE",       0xFFAAAAAA.toInt())
            DroneController.DroneStatus.TAKING_OFF      -> Pair("TAKEOFF",    0xFFFFC107.toInt())
            DroneController.DroneStatus.HOVERING        -> Pair("HOVER",      0xFF4CAF50.toInt())
            DroneController.DroneStatus.NAVIGATING      -> Pair("NAV",        0xFF2196F3.toInt())
            DroneController.DroneStatus.LANDING         -> Pair("LAND",       0xFFFF9800.toInt())
            DroneController.DroneStatus.RETURNING_HOME  -> Pair("RTH",        0xFFFF9800.toInt())
            DroneController.DroneStatus.MANUAL_OVERRIDE -> Pair("MANUAL",     0xFFF44336.toInt())
            DroneController.DroneStatus.ABORTING        -> Pair("ABORT",      0xFFF44336.toInt())
        }
        statusTv.text = label
        statusTv.setTextColor(color)
    }

    // ==================== End Drone Status View ====================

    /**
     * On Android 11+ the app needs MANAGE_EXTERNAL_STORAGE to write outside its
     * private directories (SD card root, Documents).  The permission is declared
     * in the manifest but must be toggled by the user in Settings.
     */
    private fun ensureManageExternalStoragePermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R && !Environment.isExternalStorageManager()) {
            Log.w(TAG, "MANAGE_EXTERNAL_STORAGE not granted — requesting…")
            try {
                val intent = Intent(Settings.ACTION_MANAGE_ALL_FILES_ACCESS_PERMISSION)
                startActivity(intent)
            } catch (e: Exception) {
                Log.e(TAG, "Cannot open MANAGE_ALL_FILES_ACCESS_PERMISSION settings: ${e.message}")
            }
        }
    }

    /**
     * Copy DJI SDK-managed TXT flight records into the WildBridge DJI_FlightRecords folder.
     * Runs on a background thread. Already-copied files are skipped (by filename).
     */
    private fun syncDjiFlightLogsInBackground() {
        Thread {
            try {
                val djiPath = File(getExternalFilesDir(null), "DJI/FlightRecord").absolutePath
                val count = WildBridgeFlightLogger.syncDjiFlightLogs(djiPath)
                if (count > 0) {
                    mainHandler.post {
                        Toast.makeText(this, "Synced $count DJI flight log(s) to WildBridge folder", Toast.LENGTH_SHORT).show()
                    }
                }
            } catch (e: Exception) {
                Log.w(TAG, "syncDjiFlightLogsInBackground: ${e.message}")
            }
        }.start()
    }

    private fun updateAltitudeView(altitudeMetres: Double) {
        findViewById<TextView>(R.id.text_altitude)?.text = "ALT ${altitudeMetres.toInt()}m"
    }

    private fun setupDroneNameDisplay() {
        // Find the TextView in the layout
        val droneNameText = findViewById<TextView>(R.id.text_drone_name)
        droneNameText?.let {
            // Set initial text
            it.text = droneName
            
            // Make it clickable to change drone name
            it.setOnClickListener {
                showDroneNameDialog(isFirstTime = false)
            }
        }

        findViewById<ImageButton>(R.id.button_wildbridge_settings)?.setOnClickListener { anchor ->
            showWildBridgeSettingsMenu(anchor)
        }
    }

    private fun showWildBridgeSettingsMenu(anchor: android.view.View) {
        val sdCardStatus = getDroneStorageStatus(CameraStorageLocation.SDCARD, "SD card")
        val internalStatus = getDroneStorageStatus(CameraStorageLocation.INTERNAL, "Internal")
        PopupMenu(this, anchor).apply {
            menu.add(0, 1, 0, "Change Drone Name")
            menu.add(0, 20, 1, "Configure Stream/WebRTC...")
            menu.add(0, 21, 2, detectionMenuLabel()).apply {
                isCheckable = true
                isChecked = isDetectionActiveForUi()
            }
            menu.add(0, 10, 3, "Detection Settings...")
            var nextOrder = 4
            menu.add(0, 3, nextOrder++, "Format ${sdCardStatus.menuLabel}")
            menu.add(0, 4, nextOrder, "Format ${internalStatus.menuLabel}")
            setOnMenuItemClickListener { item -> handleWildBridgeMenuItem(item.itemId) }
            show()
        }
    }
    
    private fun updateDroneNameDisplay() {
        val droneNameText = findViewById<TextView>(R.id.text_drone_name)
        droneNameText?.text = droneName
    }

    private fun setupKeyListeners() {
        setupBatteryAndRthListeners()
        setupStorageListeners()
        setupFlightStateListeners()
        setupTelemetryListeners()
    }

    private fun setupBatteryAndRthListeners() {
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

    private fun setupStorageListeners() {
        KeyManager.getInstance().listen(cameraStorageInfosKey, this) { _, newValue ->
            if (isSdCardInserted(newValue)) {
                preferSdCardStorage(newValue)
            }
        }
    }

    private fun setupFlightStateListeners() {
        // Keep isAirborne in DroneController in sync with FC telemetry — used by
        // VirtualStickVM to gate manual-override detection: only fire when airborne
        // (prevents ground-level RC drift false-positives) or during autonomous flight.
        KeyManager.getInstance().listen(isFlyingKey, this) { _, newValue ->
            val flying = newValue ?: false
            val wasFlying = DroneController.isAirborne
            DroneController.isAirborne = flying
            mainHandler.post { updateDroneStatusView(DroneController.droneStatus) }
            // Flight log session lifecycle: open a new file on takeoff, close it on landing.
            if (!wasFlying && flying) {
                WildBridgeFlightLogger.startSession()
                // Start AutoSensing on takeoff if DJI onboard detections are selected.
                if (activeDetectionSource() == DetectionSource.DJI_ONBOARD && !isAutoSensingActive) {
                    startAutoSensing()
                }
            } else if (wasFlying && !flying) {
                // 10-second grace period before closing in case of brief mid-air telemetry glitch.
                mainHandler.postDelayed({
                    if (!DroneController.isAirborne) {
                        WildBridgeFlightLogger.endSession("landed")
                        // Sync DJI TXT records — idempotent, safe to run immediately.
                        // Any file the SDK hasn't finalised yet will be picked up next launch.
                        syncDjiFlightLogsInBackground()
                    }
                }, 10_000L)
            }
        }
        setupRthModeOverrideListener()
    }

    private fun setupRthModeOverrideListener() {
        // Detect RTH triggered from the RC controller (not from our server HTTP request).
        // When the server triggers RTH it calls startReturnToHome() which sets droneStatus
        // to RETURNING_HOME BEFORE the DJI SDK switches to GO_HOME flight mode.
        // If we see GO_HOME but our status is not RETURNING_HOME, the pilot pressed the
        // RTH button on the physical controller → activate manual override so the server
        // cannot accidentally interfere with the returning drone.
        KeyManager.getInstance().listen(flightModeKey, this) { _, newValue ->
            if (newValue == FlightMode.GO_HOME &&
                DroneController.droneStatus != DroneController.DroneStatus.RETURNING_HOME) {
                mainHandler.post { DroneController.activateManualOverride() }
            }
        }
    }

    private fun setupTelemetryListeners() {
        // Keep altitude display in sync with every position update
        KeyManager.getInstance().listen(location3DKey, this) { _, newValue ->
            mainHandler.post { updateAltitudeView(newValue?.altitude ?: 0.0) }
            rebuildTelemetryCache()
        }
        // High-frequency keys: rebuild cache on every SDK push
        KeyManager.getInstance().listen(attitudeKey, this) { _, _ -> rebuildTelemetryCache() }
        KeyManager.getInstance().listen(compassHeadKey, this) { _, _ -> rebuildTelemetryCache() }
        KeyManager.getInstance().listen(flightSpeedKey, this) { _, _ -> rebuildTelemetryCache() }
        KeyManager.getInstance().listen(batteryKey, this) { _, _ -> rebuildTelemetryCache() }
    }
    
    private fun loadDroneName() {
        val storedName = sharedPreferences.getString(PREF_DRONE_NAME, DEFAULT_DRONE_NAME)?.trim().orEmpty()
        droneName = storedName.ifEmpty { DEFAULT_DRONE_NAME }

        if (storedName.isEmpty()) {
            // Persist a safe fallback to avoid generating malformed URLs like //whip.
            sharedPreferences.edit().putString(PREF_DRONE_NAME, droneName).apply()
        }

        if (storedName.isEmpty()) {
            // First time - prompt user for drone name
            mainHandler.post {
                showDroneNameDialog(isFirstTime = true)
            }
        } else {
            Log.i(TAG, "Loaded drone name: $droneName")
            WildBridgeFlightLogger.setDroneName(droneName)
        }
    }
    
    private fun showDroneNameDialog(isFirstTime: Boolean = false) {
        val input = EditText(this)
        input.hint = "e.g., drone_01, alpha, scout"
        if (!isFirstTime) {
            input.setText(droneName)
        }
        
        val builder = AlertDialog.Builder(this)
            .setTitle(if (isFirstTime) "Drone Name" else "Change Drone Name")
            .setMessage(if (isFirstTime) "Please enter a unique name for this drone:" else "Enter new name for this drone:")
            .setView(input)
            .setPositiveButton("Save") { _, _ ->
                val name = input.text.toString().trim()
                if (name.isNotEmpty()) {
                    droneName = name
                    sharedPreferences.edit().putString(PREF_DRONE_NAME, droneName).apply()
                    WildBridgeFlightLogger.setDroneName(droneName)
                    Log.i(TAG, "Drone name set to: $droneName")
                    Toast.makeText(this, "Drone name saved: $droneName", Toast.LENGTH_SHORT).show()
                    updateDroneNameDisplay()
                } else {
                    droneName = DEFAULT_DRONE_NAME
                    sharedPreferences.edit().putString(PREF_DRONE_NAME, droneName).apply()
                    WildBridgeFlightLogger.setDroneName(droneName)
                    Toast.makeText(this, "Using default name: $droneName", Toast.LENGTH_SHORT).show()
                    updateDroneNameDisplay()
                }
            }
        
        if (isFirstTime) {
            builder.setCancelable(false)
        } else {
            builder.setNegativeButton("Cancel", null)
        }
        
        builder.show()
    }

    private fun showMediamtxServerDialog() {
        val input = EditText(this)
        val current = sharedPreferences.getString(PREF_MEDIAMTX_SERVER, "").orEmpty()
        input.hint = "host o host:puerto (ej: 10.233.132.21:8889)"
        input.setText(current)

        AlertDialog.Builder(this)
            .setTitle("WHIP / mediamtx server")
            .setMessage("Opcional: si se deja vacío, se usa la IP del primer cliente de telemetría.")
            .setView(input)
            .setPositiveButton("Save") { _, _ ->
                val value = input.text.toString().trim()
                sharedPreferences.edit().putString(PREF_MEDIAMTX_SERVER, value).apply()
                val shown = if (value.isEmpty()) "auto (client IP)" else value
                Log.i(TAG, "Mediamtx server set to: $shown")
                Toast.makeText(this, "WHIP server: $shown", Toast.LENGTH_SHORT).show()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun showWebRTCFpsDialog() {
        val currentFps = getWebRTCFps()
        val labels = WEBRTC_FPS_OPTIONS.map { "$it fps" }.toTypedArray()
        val checkedIndex = WEBRTC_FPS_OPTIONS.indexOf(currentFps).coerceAtLeast(0)

        AlertDialog.Builder(this)
            .setTitle("WebRTC frame rate")
            .setSingleChoiceItems(labels, checkedIndex) { dialog, which ->
                val selectedFps = WEBRTC_FPS_OPTIONS[which]
                sharedPreferences.edit().putInt(PREF_WEBRTC_FPS, selectedFps).apply()
                webRTCStreamer?.changeMediaOptions(buildWebRTCOptions())
                Toast.makeText(this, "WebRTC FPS: $selectedFps", Toast.LENGTH_SHORT).show()
                Log.i(TAG, "WebRTC frame rate set to $selectedFps fps")
                dialog.dismiss()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun showWebRTCResolutionDialog() {
        val presets = StreamResolutionPreset.entries.toTypedArray()
        val labels = presets.map {
            if (it.width > 0 && it.height > 0) "${it.menuLabel} (${it.width}x${it.height})" else it.menuLabel
        }.toTypedArray()
        val checkedIndex = presets.indexOf(getWebRTCResolutionPreset()).coerceAtLeast(0)

        AlertDialog.Builder(this)
            .setTitle("WebRTC resolution")
            .setSingleChoiceItems(labels, checkedIndex) { dialog, which ->
                val selectedPreset = presets[which]
                sharedPreferences.edit().putString(PREF_WEBRTC_RESOLUTION, selectedPreset.prefValue).apply()
                webRTCStreamer?.changeMediaOptions(buildWebRTCOptions())
                Toast.makeText(this, "WebRTC resolution: ${selectedPreset.menuLabel}", Toast.LENGTH_SHORT).show()
                Log.i(
                    TAG,
                    "WebRTC resolution set to ${if (selectedPreset.width > 0 && selectedPreset.height > 0) "${selectedPreset.width}x${selectedPreset.height}" else "native source"}"
                )
                dialog.dismiss()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun showVideoSourceDialog() {
        val sources = VideoSourceMode.entries.toTypedArray()
        val labels = sources.map { source ->
            when (source) {
                VideoSourceMode.DJI -> "Drone camera"
                VideoSourceMode.PHONE -> "Phone back camera"
                VideoSourceMode.MOCK -> "Mock MP4"
            }
        }.toTypedArray()
        val checkedIndex = sources.indexOf(getVideoSourceMode()).coerceAtLeast(0)

        AlertDialog.Builder(this)
            .setTitle("Video source")
            .setSingleChoiceItems(labels, checkedIndex) { dialog, which ->
                setVideoSourceMode(sources[which])
                dialog.dismiss()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun showStreamSettingsDialog() {
        val configuredServer = sharedPreferences.getString(PREF_MEDIAMTX_SERVER, "")?.trim().orEmpty()
        val serverLabel = configuredServer.ifEmpty { "Auto" }
        val rows = listOf(
            SettingsActionRow("WHIP server", serverLabel),
            SettingsActionRow("Video source", getVideoSourceMode().menuLabel),
            SettingsActionRow("WebRTC FPS", "${getWebRTCFps()} fps"),
            SettingsActionRow("WebRTC resolution", getWebRTCResolutionPreset().menuLabel)
        )

        AlertDialog.Builder(this)
            .setTitle("Stream/WebRTC")
            .setAdapter(actionRowAdapter(rows)) { dialog, which ->
                dialog.dismiss()
                when (which) {
                    0 -> showMediamtxServerDialog()
                    1 -> showVideoSourceDialog()
                    2 -> showWebRTCFpsDialog()
                    3 -> showWebRTCResolutionDialog()
                }
            }
            .setNegativeButton("Close", null)
            .show()
    }

    private fun showEdgeConfidenceDialog() {
        val currentThreshold = getEdgeConfidenceThreshold()
        val labels = EDGE_CONFIDENCE_OPTIONS.map { "${(it * 100).toInt()}%" }.toTypedArray()
        val checkedIndex = EDGE_CONFIDENCE_OPTIONS.indexOfFirst { kotlin.math.abs(it - currentThreshold) < 0.001f }
            .takeIf { it >= 0 }
            ?: EDGE_CONFIDENCE_OPTIONS.indexOfFirst { kotlin.math.abs(it - DEFAULT_EDGE_CONFIDENCE_THRESHOLD) < 0.001f }.coerceAtLeast(0)

        AlertDialog.Builder(this)
            .setTitle("Edge confidence threshold")
            .setSingleChoiceItems(labels, checkedIndex) { dialog, which ->
                val selectedThreshold = EDGE_CONFIDENCE_OPTIONS[which]
                sharedPreferences.edit().putFloat(PREF_EDGE_CONFIDENCE_THRESHOLD, selectedThreshold).apply()
                if (isEdgeDetectionEnabled()) {
                    stopEdgeDetection()
                    startEdgeDetection()
                } else {
                    updateEdgeMetricsView(lastEdgeMetrics.copy(confidenceThreshold = selectedThreshold))
                }
                invalidateOptionsMenu()
                Toast.makeText(this, "Edge confidence: ${(selectedThreshold * 100).toInt()}%", Toast.LENGTH_SHORT).show()
                Log.i(TAG, "Edge confidence threshold set to $selectedThreshold")
                dialog.dismiss()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun showFormatStorageDialog(location: CameraStorageLocation, label: String) {
        val status = getDroneStorageStatus(location, label)
        AlertDialog.Builder(this)
            .setTitle("Format $label")
            .setMessage("${status.dialogText}\n\nThis deletes all media on the drone $label. Stop recording first, then continue only if you are sure.")
            .setPositiveButton("Format") { _, _ ->
                formatDroneStorage(location, label)
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun scheduleDefaultCameraRecordingConfiguration() {
        val delaysMs = longArrayOf(0L, 2_000L, 6_000L)
        delaysMs.forEach { delayMs ->
            mainHandler.postDelayed({ configureDefaultCameraRecording() }, delayMs)
        }
    }

    private fun configureDefaultCameraRecording() {
        setDefaultVideoMode()
        preferSdCardStorage(KeyManager.getInstance().getValue(cameraStorageInfosKey))
    }

    private fun setDefaultVideoMode() {
        val currentMode = KeyManager.getInstance().getValue(cameraModeKey)
        if (currentMode == CameraMode.VIDEO_NORMAL) {
            return
        }

        KeyManager.getInstance().setValue(cameraModeKey, CameraMode.VIDEO_NORMAL, object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                Log.i(TAG, "Default camera mode set to video")
            }

            override fun onFailure(error: IDJIError) {
                Log.w(TAG, "Could not set default camera mode to video: ${error.description()}")
            }
        })
    }

    private fun preferSdCardStorage(storageInfos: CameraStorageInfos?) {
        if (!isSdCardInserted(storageInfos)) {
            Log.i(TAG, "SD card storage not selected: SD card is not inserted")
            return
        }

        val currentLocation = KeyManager.getInstance().getValue(cameraStorageLocationKey)
        if (currentLocation == CameraStorageLocation.SDCARD) {
            return
        }

        KeyManager.getInstance().setValue(cameraStorageLocationKey, CameraStorageLocation.SDCARD, object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                Log.i(TAG, "Default camera storage set to SD card")
            }

            override fun onFailure(error: IDJIError) {
                Log.w(TAG, "Could not set default camera storage to SD card: ${error.description()}")
            }
        })
    }

    private fun isSdCardInserted(storageInfos: CameraStorageInfos?): Boolean {
        return storageInfos
            ?.cameraStorageInfoList
            ?.firstOrNull { it.storageType == CameraStorageLocation.SDCARD }
            ?.storageState == SDCardLoadState.INSERTED
    }

    private fun getDroneStorageStatus(location: CameraStorageLocation, label: String): DroneStorageStatus {
        val storageInfos: CameraStorageInfos? = KeyManager.getInstance().getValue(cameraStorageInfosKey)
        val info = storageInfos?.cameraStorageInfoList?.firstOrNull { it.storageType == location }
        val parts = listOfNotNull(
            info?.getStorageLeftCapacity()?.takeIf { it >= 0 }?.let { "${formatCapacity(it)} free" },
            info?.getStorageState()?.name?.takeIf { it.isNotBlank() && it != "UNKNOWN" },
            info?.getAvailableVideoDuration()?.takeIf { it >= 0 }?.let { "video ${formatDuration(it)}" }
        )
        return DroneStorageStatus(label, parts.ifEmpty { listOf("status unavailable") }.joinToString(", "))
    }

    private fun formatCapacity(megabytes: Int): String {
        return if (megabytes >= 1024) {
            String.format(java.util.Locale.US, "%.1f GB", megabytes / 1024.0)
        } else {
            "$megabytes MB"
        }
    }

    private fun formatDuration(seconds: Int): String {
        val hours = seconds / 3600
        val minutes = (seconds % 3600) / 60
        return if (hours > 0) "${hours}h ${minutes}m" else "${minutes}m"
    }

    private fun formatDroneStorage(location: CameraStorageLocation, label: String) {
        val key = KeyTools.createKey(CameraKey.KeyFormatStorage, ComponentIndexType.LEFT_OR_MAIN)
        KeyManager.getInstance().performAction(key, location, object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(result: EmptyMsg?) {
                mainHandler.post {
                    Toast.makeText(this@WildBridgeDefaultLayoutActivity, "$label formatted", Toast.LENGTH_LONG).show()
                }
                Log.i(TAG, "Formatted drone $label")
            }

            override fun onFailure(error: IDJIError) {
                val message = "Failed to format $label: ${error.description()}"
                mainHandler.post {
                    Toast.makeText(this@WildBridgeDefaultLayoutActivity, message, Toast.LENGTH_LONG).show()
                }
                Log.e(TAG, message)
            }
        })
    }

    private fun buildWhipUrl(clientIp: String): String {
        val safeDroneName = droneName.trim().ifEmpty {
            DEFAULT_DRONE_NAME
        }

        val configuredServer = sharedPreferences.getString(PREF_MEDIAMTX_SERVER, "")
            ?.trim()
            .orEmpty()

        val hostAndPort = if (configuredServer.isEmpty()) {
            "$clientIp:$MEDIAMTX_WHIP_PORT"
        } else {
            var normalized = configuredServer
                .removePrefix("http://")
                .removePrefix("https://")
                .trimEnd('/')
            if (!normalized.contains(':')) {
                normalized = "$normalized:$MEDIAMTX_WHIP_PORT"
            }
            normalized
        }

        return "http://$hostAndPort/$safeDroneName/whip"
    }

    private fun startLocationUpdates() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // Request permissions if not granted
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.ACCESS_FINE_LOCATION), 1)
            return
        }
        try {
            locationManager?.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 1f, locationListener)
            locationManager?.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 1000L, 1f, locationListener)
        } catch (e: Exception) {
            Log.e(TAG, "Error requesting location updates: ${e.message}")
        }
    }

    private fun startSensorUpdates() {
        sensorManager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.also { accelerometer ->
            sensorManager?.registerListener(sensorListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL)
        }
        sensorManager?.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)?.also { magneticField ->
            sensorManager?.registerListener(sensorListener, magneticField, SensorManager.SENSOR_DELAY_NORMAL)
        }
        sensorManager?.getDefaultSensor(Sensor.TYPE_PRESSURE)?.also { pressure ->
            sensorManager?.registerListener(sensorListener, pressure, SensorManager.SENSOR_DELAY_NORMAL)
        }
    }
    
    private fun updateOrientationAngles() {
        // Update rotation matrix, which is needed to update orientation angles.
        SensorManager.getRotationMatrix(rotationMatrix, null, accelerometerReading, magnetometerReading)
        // "rotationMatrix" now has up-to-date information.

        SensorManager.getOrientation(rotationMatrix, orientationAngles)
        // "orientationAngles" now has up-to-date information.
        
        // Convert azimuth to degrees (0-360)
        var azimuth = Math.toDegrees(orientationAngles[0].toDouble())
        if (azimuth < 0) {
            azimuth += 360.0
        }
        phoneHeading = azimuth
    }

    private fun startServers() {
        val deviceIp = getDeviceIpAddress()
        
        // Start mDNS/Zeroconf service registration (RECOMMENDED for discovery)
        registerMdnsService()
        
        // Start Discovery Server (UDP broadcast/multicast fallback)
        startDiscoveryServer()
        
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
                telemetryServer?.onFirstClientConnected = { clientIp ->
                    val whipUrl = buildWhipUrl(clientIp)
                    Log.i(TAG, "First telemetry client from $clientIp — starting WHIP: $whipUrl")
                    Thread { startWhipPublishing(whipUrl) }.start()
                }
                telemetryServer?.start()
                Log.i(TAG, "Telemetry server started on $deviceIp:$TELEMETRY_PORT")
            } catch (e: Exception) {
                Log.e(TAG, "Error starting telemetry server: ${e.message}")
            }
        } else {
            Log.w(TAG, "Telemetry port $TELEMETRY_PORT already in use")
        }

        // WebRTC video via WHIP — create the shared frame source/publisher.
        // WHIP publishing starts automatically when bridge connects to telemetry.
        try {
            webRTCStreamer = WebRTCStreamer(
                context = applicationContext,
                cameraIndex = ComponentIndexType.LEFT_OR_MAIN,
                droneName = droneName,
                options = buildWebRTCOptions(),
                mockVideoEnabled = isMockVideoEnabled()
            )
            webRTCStreamer?.setVideoSourceMode(getVideoSourceMode())
            webRTCStreamer?.listener = object : WebRTCStreamer.WebRTCStreamerListener {
                override fun onServerStarted(ip: String, port: Int) {
                    Log.i(TAG, "WHIP publishing from $ip")
                }
                override fun onServerStopped() {
                    Log.i(TAG, "WebRTC streamer stopped")
                }
                override fun onServerError(error: String) {
                    Log.e(TAG, "WebRTC error: $error")
                }
                override fun onMetrics(metrics: WebRTCStreamMetrics) {
                    lastWebRTCMetrics = metrics
                    rebuildTelemetryCache()
                    mainHandler.post { updateWebRTCMetricsView(metrics) }
                }
            }
            Log.i(TAG, "WebRTC streamer ready (WHIP starts on first telemetry client)")

            // If the telemetry callback already fired before streamer was ready, start now
            val pendingUrl = lastWhipUrl
            if (pendingUrl != null) {
                Log.i(TAG, "Deferred WHIP start: $pendingUrl")
                Thread { startWhipPublishing(pendingUrl) }.start()
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error creating WebRTC streamer: ${e.message}")
        }
    }

    private fun showServerInfo() {
        val deviceIp = getDeviceIpAddress() ?: "Unknown"
        val message = """
            WildBridge Servers Started
            IP: $deviceIp
            HTTP Commands: $HTTP_PORT
            Telemetry: $TELEMETRY_PORT
            Video: WHIP (auto on bridge connect)
        """.trimIndent()
        
        Toast.makeText(this, message, Toast.LENGTH_LONG).show()
        Log.i(TAG, message)
    }

    override fun onDestroy() {
        detachDefaultLayoutHsiWidgets()

        try {
            // Stop AutoSensing
            stopAutoSensing()

            stopMockVideoPreview()
            stopEdgeDetection()

            // Stop all servers
            httpServer?.stop()
            telemetryServer?.onFirstClientConnected = null
            telemetryServer?.stop()
            webRTCStreamer?.listener = null
            webRTCStreamer?.stop()
            stopDiscoveryServer()
            httpServer = null
            telemetryServer = null
            webRTCStreamer = null

            // Unregister mDNS service
            unregisterMdnsService()

            // Stop location updates
            locationManager?.removeUpdates(locationListener)

            // Stop sensor updates
            sensorManager?.unregisterListener(sensorListener)

            // Release Multicast Lock
            if (multicastLock?.isHeld == true) {
                multicastLock?.release()
            }

            // Cancel key listeners
            KeyManager.getInstance().cancelListen(this)

            // Clean up DroneController listeners and resources
            DroneController.manualOverrideListener = null
            DroneController.droneStatusListener = null
            DroneController.destroy()

            // Close the active flight log if the app is killed mid-flight
            WildBridgeFlightLogger.endSession("app_stopped")

            mainHandler.removeCallbacksAndMessages(null)
            stopPhoneCameraPreview()

            Log.i(TAG, "All servers stopped")
        } finally {
            super.onDestroy()
        }
    }

    private fun detachDefaultLayoutHsiWidgets() {
        try {
            val hsiWidget = horizontalSituationIndicatorWidget ?: return
            val parent = hsiWidget.parent as? ViewGroup ?: return
            parent.removeView(hsiWidget)
        } catch (e: Exception) {
            Log.w(TAG, "Failed to detach HSI widgets during destroy: ${e.message}")
        }
    }
    
    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        menu.add(0, 1, 0, "Change Drone Name")
        menu.add(0, 20, 1, "Configure Stream/WebRTC...")
        menu.add(0, 21, 2, detectionMenuLabel()).apply {
            isCheckable = true
            isChecked = isDetectionActiveForUi()
        }
        menu.add(0, 10, 3, "Detection Settings...")
        var nextOrder = 4
        menu.add(0, 3, nextOrder++, "Format Drone SD Card")
        menu.add(0, 4, nextOrder, "Format Drone Internal Storage")
        return super.onCreateOptionsMenu(menu)
    }
    
    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        return if (handleWildBridgeMenuItem(item.itemId)) true else super.onOptionsItemSelected(item)
    }

    private fun handleWildBridgeMenuItem(itemId: Int): Boolean {
        val action = when (itemId) {
            1 -> { { showDroneNameDialog(isFirstTime = false) } }
            2, 5, 7, 9, 20 -> ::showStreamSettingsDialog
            21 -> { { setDetectionsEnabled(!isDetectionActiveForUi()) } }
            8, 10 -> ::showDetectionSettingsDialog
            11 -> { { showEdgeFilePicker(REQUEST_EDGE_MODEL_FILE, "Select YOLO TFLite model") } }
            12 -> { { showEdgeFilePicker(REQUEST_EDGE_LABELS_FILE, "Select model labels") } }
            13 -> ::showEdgeConfidenceDialog
            6 -> ::showVideoSourceDialog
            3 -> { { showFormatStorageDialog(CameraStorageLocation.SDCARD, "SD card") } }
            4 -> { { showFormatStorageDialog(CameraStorageLocation.INTERNAL, "internal storage") } }
            else -> return false
        }
        action()
        return true
    }

    // ==================== Utility Methods ====================

    private fun startDiscoveryServer() {
        if (isDiscoveryRunning) return
        isDiscoveryRunning = true
        
        // Thread 1: Handle broadcast/unicast UDP on port 30000
        discoveryThread = thread(start = true) {
            try {
                // Create socket that can receive broadcast packets
                discoverySocket = DatagramSocket(null)
                discoverySocket?.reuseAddress = true
                discoverySocket?.broadcast = true
                discoverySocket?.bind(java.net.InetSocketAddress("0.0.0.0", DISCOVERY_PORT))
                
                val buffer = ByteArray(1024)
                Log.i(TAG, "✓ Discovery server started on 0.0.0.0:$DISCOVERY_PORT (broadcast enabled)")
                
                while (isDiscoveryRunning) {
                    val packet = DatagramPacket(buffer, buffer.size)
                    discoverySocket?.receive(packet)
                    val message = String(packet.data, 0, packet.length).trim()
                    
                    Log.d(TAG, "📡 UDP from ${packet.address.hostAddress}:${packet.port}: $message")
                    
                    if (message == DISCOVERY_MSG) {
                        respondToDiscovery(packet.address, packet.port)
                    }
                }
            } catch (e: Exception) {
                if (isDiscoveryRunning) {
                    Log.e(TAG, "Discovery server error: ${e.message}")
                }
            } finally {
                discoverySocket?.close()
                discoverySocket = null
                Log.i(TAG, "Discovery server stopped")
            }
        }
        
        // Thread 2: Handle multicast on 239.255.42.99:30001
        multicastThread = thread(start = true) {
            try {
                multicastSocket = MulticastSocket(MULTICAST_PORT)
                multicastSocket?.reuseAddress = true
                
                val group = InetAddress.getByName(MULTICAST_GROUP)
                multicastSocket?.joinGroup(group)
                
                val buffer = ByteArray(1024)
                Log.i(TAG, "✓ Multicast discovery started on $MULTICAST_GROUP:$MULTICAST_PORT")
                
                while (isDiscoveryRunning) {
                    val packet = DatagramPacket(buffer, buffer.size)
                    multicastSocket?.receive(packet)
                    val message = String(packet.data, 0, packet.length).trim()
                    
                    Log.d(TAG, "📡 Multicast from ${packet.address.hostAddress}: $message")
                    
                    if (message == DISCOVERY_MSG) {
                        // Respond to multicast discovery
                        respondToDiscovery(packet.address, MULTICAST_PORT)
                    }
                }
                
                multicastSocket?.leaveGroup(group)
            } catch (e: Exception) {
                if (isDiscoveryRunning) {
                    Log.e(TAG, "Multicast discovery error: ${e.message}")
                }
            } finally {
                multicastSocket?.close()
                multicastSocket = null
                Log.i(TAG, "Multicast discovery stopped")
            }
        }
    }
    
    private fun respondToDiscovery(senderAddress: InetAddress, senderPort: Int) {
        val deviceIp = getDeviceIpAddress()
        Log.i(TAG, "🔍 Discovery request from ${senderAddress.hostAddress}. My IP: $deviceIp")
        
        if (deviceIp != null) {
            val response = "$DISCOVERY_RESPONSE_PREFIX$deviceIp:$droneName"
            val responseData = response.toByteArray()
            
            try {
                // Send response back to sender
                val responsePacket = DatagramPacket(
                    responseData,
                    responseData.size,
                    senderAddress,
                    senderPort
                )
                
                // Try using the socket that received the message
                val socketToUse = if (senderPort == MULTICAST_PORT) multicastSocket else discoverySocket
                socketToUse?.send(responsePacket)
                
                Log.i(TAG, "✓ Sent discovery response to ${senderAddress.hostAddress}:$senderPort → $response")
            } catch (e: Exception) {
                Log.e(TAG, "Failed to send discovery response: ${e.message}")
            }
        }
    }

    private fun stopDiscoveryServer() {
        isDiscoveryRunning = false
        try {
            discoverySocket?.close()
            multicastSocket?.close()
        } catch (e: Exception) {
            // Ignore
        }
        try {
            discoveryThread?.join(1000)
            multicastThread?.join(1000)
        } catch (e: Exception) {
            // Ignore
        }
        Log.i(TAG, "All discovery servers stopped")
    }
    
    // ==================== mDNS/Zeroconf Service Registration ====================
    
    private val registrationListener = MdnsRegistrationListener(this)

    private class MdnsRegistrationListener(activity: WildBridgeDefaultLayoutActivity) : NsdManager.RegistrationListener {
        private val activityRef = WeakReference(activity)

        override fun onServiceRegistered(serviceInfo: NsdServiceInfo) {
            activityRef.get()?.onMdnsServiceRegistered(serviceInfo)
        }

        override fun onRegistrationFailed(serviceInfo: NsdServiceInfo, errorCode: Int) {
            activityRef.get()?.onMdnsRegistrationFailed(errorCode)
        }

        override fun onServiceUnregistered(serviceInfo: NsdServiceInfo) {
            activityRef.get()?.onMdnsServiceUnregistered(serviceInfo)
        }

        override fun onUnregistrationFailed(serviceInfo: NsdServiceInfo, errorCode: Int) {
            activityRef.get()?.onMdnsUnregistrationFailed(errorCode)
        }
    }
    
    private fun registerMdnsService() {
        try {
            nsdManager = applicationContext.getSystemService(Context.NSD_SERVICE) as NsdManager
            
            val serviceInfo = NsdServiceInfo().apply {
                // Service name will be the drone name (e.g., "cacatua")
                serviceName = droneName
                serviceType = MDNS_SERVICE_TYPE
                port = HTTP_PORT
                
                // Add service attributes (TXT records)
                setAttribute("name", droneName)
                setAttribute("serial", droneSerialNumber)
                setAttribute("http", HTTP_PORT.toString())
                setAttribute("telemetry", TELEMETRY_PORT.toString())
                setAttribute("video", "whip")
            }
            
            isMdnsRegistrationRequested = true
            nsdManager?.registerService(
                serviceInfo,
                NsdManager.PROTOCOL_DNS_SD,
                registrationListener
            )
            
            Log.i(TAG, "Registering mDNS service: $droneName.$MDNS_SERVICE_TYPE")
            
        } catch (e: Exception) {
            Log.e(TAG, "Failed to register mDNS service: ${e.message}")
            isMdnsRegistrationRequested = false
            isMdnsRegistered = false
            nsdManager = null
        }
    }

    private fun onMdnsServiceRegistered(serviceInfo: NsdServiceInfo) {
        mdnsServiceName = serviceInfo.serviceName
        isMdnsRegistrationRequested = false
        isMdnsRegistered = true
        Log.i(TAG, "✓ mDNS service registered: ${serviceInfo.serviceName} (${MDNS_SERVICE_TYPE})")
    }

    private fun onMdnsRegistrationFailed(errorCode: Int) {
        Log.e(TAG, "✗ mDNS registration failed: error $errorCode")
        isMdnsRegistrationRequested = false
        isMdnsRegistered = false
    }

    private fun onMdnsServiceUnregistered(serviceInfo: NsdServiceInfo) {
        Log.i(TAG, "mDNS service unregistered: ${serviceInfo.serviceName}")
        isMdnsRegistrationRequested = false
        isMdnsRegistered = false
        mdnsServiceName = null
        nsdManager = null
    }

    private fun onMdnsUnregistrationFailed(errorCode: Int) {
        Log.e(TAG, "mDNS unregistration failed: error $errorCode")
        isMdnsRegistrationRequested = false
        isMdnsRegistered = false
        mdnsServiceName = null
        nsdManager = null
    }
    
    private fun unregisterMdnsService() {
        if (isMdnsRegistered || isMdnsRegistrationRequested) {
            try {
                nsdManager?.unregisterService(registrationListener)
                Log.i(TAG, "Unregistering mDNS service")
            } catch (e: Exception) {
                Log.e(TAG, "Error unregistering mDNS: ${e.message}")
            } finally {
                isMdnsRegistrationRequested = false
                isMdnsRegistered = false
                mdnsServiceName = null
                nsdManager = null
            }
        }
    }
    
    private fun fetchDroneSerialNumber() {
        try {
            // Get drone serial number from DJI SDK
            val serialKey = KeyTools.createKey(FlightControllerKey.KeySerialNumber)
            KeyManager.getInstance().getValue(serialKey, object : dji.v5.common.callback.CommonCallbacks.CompletionCallbackWithParam<String> {
                override fun onSuccess(serialNumber: String?) {
                    droneSerialNumber = serialNumber?.takeLast(8) ?: "UNKNOWN"
                    Log.i(TAG, "Drone serial number: $droneSerialNumber")
                }
                override fun onFailure(error: dji.v5.common.error.IDJIError) {
                    droneSerialNumber = "UNKNOWN"
                    Log.w(TAG, "Failed to get drone serial: ${error.description()}")
                }
            })
        } catch (e: Exception) {
            droneSerialNumber = "UNKNOWN"
            Log.e(TAG, "Error fetching drone serial: ${e.message}")
        }
    }

    private fun getDeviceIpAddress(): String? {
        return try {
            val candidates = deviceIpCandidates()
            candidates.forEach {
                Log.d(TAG, "Found IP: ${it.ip} on interface: ${it.interfaceName}")
            }
            (candidates.firstOrNull { it.isPreferred } ?: candidates.firstOrNull())?.ip
        } catch (e: SocketException) {
            Log.e(TAG, "Error getting IP address: ${e.message}")
            null
        }
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

    private fun getLocation3D(): LocationCoordinate3D = location3DKey.get(LocationCoordinate3D(0.0, 0.0, .0))
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
        val shouldLatchHomePoint = !isHomePointSetLatch && !isFlyingKey.get(false) && run {
            val home = getHomeLocation()
            val hasHomeCoordinates = home.latitude != 0.0 && home.longitude != 0.0
            if (!hasHomeCoordinates) {
                false
            } else {
                val current = getLocation3D()
                val distance = DroneController.calculateDistance(
                    current.latitude, current.longitude,
                    home.latitude, home.longitude
                )
                distance < 0.5
            }
        }

        if (shouldLatchHomePoint) {
            isHomePointSetLatch = true
        }

        return isHomePointSetLatch
    }

    private fun getTelemetryJson(): String = cachedTelemetryJson

    private fun rebuildTelemetryCache() {
        cachedTelemetryJson = buildTelemetryJson()
    }

    private fun escapeJson(value: String): String {
        return value.replace("\\", "\\\\").replace("\"", "\\\"")
    }

    private fun detectionTelemetryJson(): String {
        val selectedSource = getDetectionSource()
        val source = activeDetectionSource()
        val active = when (source) {
            DetectionSource.NONE -> false
            DetectionSource.DJI_ONBOARD -> isAutoSensingActive
            DetectionSource.YOLO_ON_PHONE -> edgeDetectionController != null
        }
        val modelName = if (selectedSource == DetectionSource.YOLO_ON_PHONE) {
            sharedPreferences.getString(PREF_EDGE_MODEL_NAME, null)
        } else {
            null
        }
        val labelsName = if (selectedSource == DetectionSource.YOLO_ON_PHONE) {
            sharedPreferences.getString(PREF_EDGE_LABELS_NAME, null)
        } else {
            null
        }
        val thresholdJson = if (selectedSource == DetectionSource.YOLO_ON_PHONE) getEdgeConfidenceThreshold().toString() else "null"
        val modelJson = modelName?.let { "\"${escapeJson(it)}\"" } ?: "null"
        val labelsJson = labelsName?.let { "\"${escapeJson(it)}\"" } ?: "null"
        val targetsJson = DetectedTarget.listToJsonArray(currentDetectedTargets)
        return """{"source":"${source.prefValue}","selectedSource":"${selectedSource.prefValue}","label":"${selectedSource.menuLabel}","enabled":${isDetectionsEnabled()},"active":$active,"count":${currentDetectedTargets.size},"model":$modelJson,"labels":$labelsJson,"confidenceThreshold":$thresholdJson,"targets":$targetsJson}"""
    }

    private fun buildTelemetryJson(): String {
        if (TelemetryProvider.isMockTelemetryEnabled()) {
            val mock = TelemetryProvider.currentMockTelemetry(droneName)
            val phoneLat = phoneLocation?.latitude ?: 0.0
            val phoneLon = phoneLocation?.longitude ?: 0.0
            val phoneBattery = batteryManager?.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY) ?: -1
            val wifiRssi = wifiManager?.connectionInfo?.rssi ?: -100
            val phoneLocationJson = """{"latitude":$phoneLat,"longitude":$phoneLon,"heading":$phoneHeading,"pressure":$phonePressure,"battery":$phoneBattery,"wifiRssi":$wifiRssi}"""
            val webRtcJson = lastWebRTCMetrics.toTelemetryJson()
            val detectionsJson = detectionTelemetryJson()

            return """{"droneName":"$droneName","speed":${mock.velocity},"heading":${mock.heading},"attitude":${mock.attitude},"location":${mock.location},"phoneLocation":$phoneLocationJson,"webRtc":$webRtcJson,"detections":$detectionsJson,"gimbalAttitude":${mock.gimbalAttitude},"gimbalJointAttitude":${mock.gimbalAttitude},"zoomFl":24,"hybridFl":24,"opticalFl":24,"zoomRatio":1.0,"batteryLevel":${mock.batteryPercent},"satelliteCount":${mock.satelliteCount},"homeLocation":{"latitude":${mock.location.latitude},"longitude":${mock.location.longitude}},"distanceToHome":0.0,"waypointReached":false,"intermediaryWaypointReached":false,"yawReached":true,"altitudeReached":true,"isRecording":true,"homeSet":true,"remainingFlightTime":1320,"timeNeededToGoHome":45,"timeNeededToLand":18,"totalTime":63,"maxRadiusCanFlyAndGoHome":900,"remainingCharge":${mock.batteryPercent},"batteryNeededToLand":12,"batteryNeededToGoHome":18,"seriousLowBatteryThreshold":10,"lowBatteryThreshold":20,"flightMode":"${mock.flightMode}","isManualOverrideActive":false,"autoSensingActive":$isAutoSensingActive,"detectedTargets":${DetectedTarget.listToJsonArray(currentDetectedTargets)}}"""
        }

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

        val phoneLat = phoneLocation?.latitude ?: 0.0
        val phoneLon = phoneLocation?.longitude ?: 0.0
        
        // Phone Status
        val phoneBattery = batteryManager?.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY) ?: -1
        val wifiRssi = wifiManager?.connectionInfo?.rssi ?: -100
        
        val phoneLocationJson = """{"latitude":$phoneLat,"longitude":$phoneLon,"heading":$phoneHeading,"pressure":$phonePressure,"battery":$phoneBattery,"wifiRssi":$wifiRssi}"""
        val webRtcJson = lastWebRTCMetrics.toTelemetryJson()
        val detectionsJson = detectionTelemetryJson()

        return """{"droneName":"$droneName","speed":$speed,"heading":$heading,"attitude":$attitude,"location":$location,"phoneLocation":$phoneLocationJson,"webRtc":$webRtcJson,"detections":$detectionsJson,"gimbalAttitude":$gimbalAttitude,"gimbalJointAttitude":$gimbalJointAttitude,"zoomFl":$zoomFl,"hybridFl":$hybridFl,"opticalFl":$opticalFl,"zoomRatio":$zoomRatio,"batteryLevel":$batteryLevel,"satelliteCount":$satelliteCount,"homeLocation":$homeLocation,"distanceToHome":$distanceToHome,"waypointReached":$waypointReached,"intermediaryWaypointReached":$intermediaryWaypointReached,"yawReached":$yawReached,"altitudeReached":$altitudeReached,"isRecording":$isRecording,"homeSet":$homeSet,"remainingFlightTime":$remainingFlightTime,"timeNeededToGoHome":$timeNeededToGoHome,"timeNeededToLand":$timeNeededToLand,"totalTime":$totalTime,"maxRadiusCanFlyAndGoHome":$maxRadiusCanFlyAndGoHome,"remainingCharge":$remainingCharge,"batteryNeededToLand":$batteryNeededToLand,"batteryNeededToGoHome":$batteryNeededToGoHome,"seriousLowBatteryThreshold":$seriousLowBatteryThreshold,"lowBatteryThreshold":$lowBatteryThreshold,"flightMode":"$flightMode","isManualOverrideActive":${DroneController.isManualOverrideActive},"autoSensingActive":$isAutoSensingActive,"detectedTargets":${DetectedTarget.listToJsonArray(currentDetectedTargets)}}"""
    }

    private inner class WildBridgeHttpCommandHandler {
        private val postRoutes: Map<String, (String) -> String> = mapOf(
            "/send/takeoff" to {
                DroneController.startTakeOff()
                "Takeoff command sent."
            },
            "/send/land" to {
                DroneController.startLanding()
                "Landing command sent."
            },
            "/send/RTH" to {
                DroneController.startReturnToHome()
                "Return to home command sent."
            },
            "/send/stick" to { postData ->
                if (DroneController.shouldRejectAutonomousCommand("stick")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    val command = WildBridgeHttpCommandParser.parseStick(postData)
                    DroneController.setStick(command.leftX, command.leftY, command.rightX, command.rightY)
                    "Received: leftX: ${command.leftX}, leftY: ${command.leftY}, " +
                        "rightX: ${command.rightX}, rightY: ${command.rightY}"
                }
            },
            "/send/gimbal/pitch" to { postData ->
                val command = WildBridgeHttpCommandParser.parseGimbal(postData)
                val rotation = GimbalAngleRotation(
                    GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                    command.pitch,
                    command.roll,
                    command.yaw,
                    false,
                    true,
                    true,
                    0.1,
                    false,
                    0
                )
                gimbalKey.action(rotation)
                "Received: roll: ${command.roll}, pitch: ${command.pitch}, yaw: ${command.yaw}"
            },
            "/send/gimbal/yaw" to { postData ->
                val command = WildBridgeHttpCommandParser.parseGimbal(postData)
                val rotation = GimbalAngleRotation(
                    GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                    command.pitch,
                    command.roll,
                    command.yaw,
                    true,
                    true,
                    false,
                    0.1,
                    false,
                    0
                )
                gimbalKey.action(rotation)
                "Received: roll: ${command.roll}, pitch: ${command.pitch}, yaw: ${command.yaw}"
            },
            "/send/gotoYaw" to { postData ->
                if (DroneController.shouldRejectAutonomousCommand("gotoYaw")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    val yaw = postData.split(",")[0].toDouble()
                    DroneController.gotoYaw(yaw)
                    "Received: yaw: $yaw"
                }
            },
            "/send/gotoAltitude" to { postData ->
                if (DroneController.shouldRejectAutonomousCommand("gotoAltitude")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    val targetAltitude = postData.split(",")[0].toDouble()
                    DroneController.gotoAltitude(targetAltitude)
                    "Received: Altitude: $targetAltitude"
                }
            },
            "/send/camera/zoom" to { postData ->
                val targetZoom = postData.toDouble()
                zoomKey.set(targetZoom)
                "Received: zoom: $targetZoom"
            },
            "/send/abortMission" to {
                DroneController.setStick(0.0f, 0.0f, 0.0f, 0.0f)
                DroneController.disableVirtualStick()
                "Received: abortMission"
            },
            "/send/abortAll" to {
                DroneController.abortAllMissions()
                "Received: abortAll"
            },
            "/send/enableVirtualStick" to {
                if (DroneController.shouldRejectAutonomousCommand("enableVirtualStick")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    DroneController.enableVirtualStick()
                    "Received: enableVirtualStick"
                }
            },
            "/send/camera/startRecording" to {
                startRecording.action()
                "Received: camera start recording"
            },
            "/send/camera/stopRecording" to {
                stopRecording.action()
                "Received: camera stop recording"
            },
            "/send/gotoWP" to { postData ->
                if (DroneController.shouldRejectAutonomousCommand("gotoWP")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    when (val command = WildBridgeHttpCommandParser.parseWaypoint(postData)) {
                        is WildBridgeHttpCommandParser.ParseResult.Invalid -> command.message
                        is WildBridgeHttpCommandParser.ParseResult.Valid -> {
                            val waypoint = command.value
                            DroneController.gotoWP(
                                waypoint.latitude,
                                waypoint.longitude,
                                waypoint.altitude
                            )
                            "Waypoint command received: Latitude=${waypoint.latitude}, " +
                                "Longitude=${waypoint.longitude}, Altitude=${waypoint.altitude}"
                        }
                    }
                }
            },
            "/send/gotoWPwithPID" to { postData ->
                if (DroneController.shouldRejectAutonomousCommand("gotoWPwithPID")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    when (val command = WildBridgeHttpCommandParser.parseWaypointPid(postData)) {
                        is WildBridgeHttpCommandParser.ParseResult.Invalid -> command.message
                        is WildBridgeHttpCommandParser.ParseResult.Valid -> {
                            val waypoint = command.value
                            DroneController.navigateToWaypointWithPID(
                                waypoint.latitude,
                                waypoint.longitude,
                                waypoint.altitude,
                                waypoint.yaw,
                                waypoint.maxSpeed
                            )
                            "Waypoint command received: Latitude=${waypoint.latitude}, " +
                                "Longitude=${waypoint.longitude}, Altitude=${waypoint.altitude}, " +
                                "Yaw=${waypoint.yaw}, MaxSpeed=${waypoint.maxSpeed}"
                        }
                    }
                }
            },
            "/send/navigateTrajectory" to { postData ->
                if (DroneController.shouldRejectAutonomousCommand("navigateTrajectory")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    Log.d("DroneServer", "Received trajectory data: $postData")
                    when (val command = WildBridgeHttpCommandParser.parseTrajectory(postData)) {
                        is WildBridgeHttpCommandParser.ParseResult.Invalid -> command.message
                        is WildBridgeHttpCommandParser.ParseResult.Valid -> {
                            val waypoints = command.value
                            Log.d("DroneServer", "Navigating trajectory with ${waypoints.size} waypoints")
                            DroneController.navigateTrajectory(waypoints)
                            "Trajectory command received. Waypoints=${waypoints.size}"
                        }
                    }
                }
            },
            "/send/navigateTrajectoryDJINative" to { postData ->
                if (DroneController.shouldRejectAutonomousCommand("navigateTrajectoryDJINative")) {
                    AUTONOMOUS_COMMAND_REJECTED
                } else {
                    when (val command = WildBridgeHttpCommandParser.parseNativeTrajectory(postData)) {
                        is WildBridgeHttpCommandParser.ParseResult.Invalid -> command.message
                        is WildBridgeHttpCommandParser.ParseResult.Valid -> {
                            val trajectory = command.value
                            DroneController.navigateTrajectoryNative(
                                trajectory.waypoints,
                                trajectory.speed
                            )
                            "DJI native mission requested with ${trajectory.waypoints.size} waypoints " +
                                "at ${trajectory.speed}m/s"
                        }
                    }
                }
            },
            "/send/abort/DJIMission" to {
                DroneController.endMission()
                "Mission stop requested"
            },
            "/send/setRTHAltitude" to { postData ->
                val altitude = postData.toIntOrNull()
                if (altitude != null) {
                    DroneController.setRTHAltitude(altitude)
                    "RTH altitude set to $altitude m"
                } else {
                    "Invalid altitude value"
                }
            },
            "/send/deactivateManualOverride" to {
                DroneController.deactivateManualOverride()
                mainHandler.post { updateManualOverrideUI() }
                "Manual override deactivated. Autonomous commands are now allowed."
            },
            "/get/isManualOverrideActive" to {
                if (DroneController.isManualOverrideActive) "true" else "false"
            },
            "/send/autoSensing/start" to {
                mainHandler.post {
                    startAutoSensing()
                    findViewById<Switch>(R.id.sw_auto_sensing)?.isChecked = true
                }
                "AutoSensing start requested"
            },
            "/send/autoSensing/stop" to {
                mainHandler.post {
                    stopAutoSensing()
                    findViewById<Switch>(R.id.sw_auto_sensing)?.isChecked = false
                }
                "AutoSensing stop requested"
            },
            "/get/autoSensing/status" to {
                """{"active":$isAutoSensingActive,"targetCount":${currentDetectedTargets.size}}"""
            },
            "/get/autoSensing/targets" to {
                DetectedTarget.listToJsonArray(currentDetectedTargets).toString()
            }
        )

        fun handlePostRequest(uri: String, postData: String): String {
            return try {
                Log.i("DroneServer", "POST $uri with data: $postData")
                postRoutes[uri]?.invoke(postData) ?: "Not Found: $uri"
            } catch (e: Exception) {
                Log.e("DroneServer", "Error processing POST request: ${e.message}", e)
                "Error processing request: ${e.message}"
            }
        }
    }

    // ==================== HTTP Server ====================

    private inner class SimpleHttpServer(private val port: Int) {
        private var serverSocket: ServerSocket? = null
        private val executor = Executors.newFixedThreadPool(10)
        private val commandHandler = WildBridgeHttpCommandHandler()
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
                try {
                    clientSocket.close()
                } catch (closeError: IOException) {
                    Log.w(
                        "SimpleHttpServer",
                        "Error closing client socket: ${closeError.message}"
                    )
                }
            }
        }

        private fun handleHttpRequest(method: String, uri: String, postData: String): String {
            return when (method) {
                "POST" -> handlePostRequest(uri, postData)
                "GET" -> handleGetRequest(uri)
                "OPTIONS" -> "OK"
                else -> "Method Not Allowed"
            }
        }
        
        private fun handleGetRequest(uri: String): String {
            return when (uri) {
                "/config" -> {
                    val deviceIp = getDeviceIpAddress() ?: "unknown"
                    """{"droneName":"$droneName","ipAddress":"$deviceIp","httpPort":$HTTP_PORT,""" +
                        """"telemetryPort":$TELEMETRY_PORT,"videoMode":"whip"}"""
                }
                else -> "Use POST for commands. Telemetry available on port $TELEMETRY_PORT. " +
                    "Config available at GET /config"
            }
        }

        private fun handlePostRequest(uri: String, postData: String): String {
            return commandHandler.handlePostRequest(uri, postData)
        }
    }
}

private data class DeviceIpCandidate(
    val ip: String,
    val interfaceName: String
) {
    val isPreferred: Boolean
        get() = interfaceName.contains("wlan") || interfaceName.contains("ap")
}

private fun deviceIpCandidates(): List<DeviceIpCandidate> {
    return Collections.list(NetworkInterface.getNetworkInterfaces())
        .filter { it.isUsableNetworkInterface() }
        .flatMap { it.ipv4Candidates() }
}

private fun NetworkInterface.isUsableNetworkInterface(): Boolean {
    return isUp && !isLoopback
}

private fun NetworkInterface.ipv4Candidates(): List<DeviceIpCandidate> {
    val interfaceName = name.lowercase()
    return Collections.list(inetAddresses)
        .filterIsInstance<Inet4Address>()
        .filterNot { it.isLoopbackAddress }
        .mapNotNull { address ->
            val ip = address.hostAddress ?: return@mapNotNull null
            DeviceIpCandidate(ip = ip, interfaceName = interfaceName)
        }
}
