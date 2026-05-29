package dji.sampleV5.aircraft.controller

import android.util.Log
import com.dji.wpmzsdk.common.data.Template
import com.dji.wpmzsdk.manager.WPMZManager
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sampleV5.aircraft.utils.wpml.WaypointInfoModel
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.sdk.wpmz.value.mission.ActionGimbalRotateParam
import dji.sdk.wpmz.value.mission.WaylineActionGroup
import dji.sdk.wpmz.value.mission.WaylineActionInfo
import dji.sdk.wpmz.value.mission.WaylineActionNodeList
import dji.sdk.wpmz.value.mission.WaylineActionTreeNode
import dji.sdk.wpmz.value.mission.WaylineActionTrigger
import dji.sdk.wpmz.value.mission.WaylineActionTriggerType
import dji.sdk.wpmz.value.mission.WaylineActionType
import dji.sdk.wpmz.value.mission.WaylineActionsRelationType
import dji.sdk.wpmz.value.mission.WaylineAltitudeMode
import dji.sdk.wpmz.value.mission.WaylineCoordinateMode
import dji.sdk.wpmz.value.mission.WaylineCoordinateParam
import dji.sdk.wpmz.value.mission.WaylineDroneInfo
import dji.sdk.wpmz.value.mission.WaylineExitOnRCLostAction
import dji.sdk.wpmz.value.mission.WaylineExitOnRCLostBehavior
import dji.sdk.wpmz.value.mission.WaylineFinishedAction
import dji.sdk.wpmz.value.mission.WaylineFlyToWaylineMode
import dji.sdk.wpmz.value.mission.WaylineGimbalActuatorRotateMode
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate2D
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate3D
import dji.sdk.wpmz.value.mission.WaylineMission
import dji.sdk.wpmz.value.mission.WaylineMissionConfig
import dji.sdk.wpmz.value.mission.WaylinePositioningType
import dji.sdk.wpmz.value.mission.WaylineTemplateWaypointInfo
import dji.sdk.wpmz.value.mission.WaylineWaypoint
import dji.sdk.wpmz.value.mission.WaylineWaypointGimbalHeadingMode
import dji.sdk.wpmz.value.mission.WaylineWaypointGimbalHeadingParam
import dji.sdk.wpmz.value.mission.WaylineWaypointPitchMode
import dji.sdk.wpmz.value.mission.WaylineWaypointTurnMode
import dji.sdk.wpmz.value.mission.WaylineWaypointYawMode
import dji.sdk.wpmz.value.mission.WaylineWaypointYawParam
import dji.sdk.wpmz.value.mission.WaylineWaypointYawPathMode
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.utils.common.ContextUtil
import java.io.File
import java.text.SimpleDateFormat
import java.util.ArrayList
import java.util.Date
import java.util.Locale
import java.util.zip.ZipEntry
import java.util.zip.ZipFile

/**
 * Helper class to encapsulate KMZ Wayline mission template generation, KMZ packing/extracting,
 * pushing waylines, and native waypoint execution flows.
 * Decouples DJI WPMZ/KMZ logic from virtual stick controls.
 */
object WaylineMissionHelper {

    @Volatile
    var lastMissionNameNoExt: String = ""
        internal set

    @Volatile
    var lastMissionKmzPath: String = ""
        internal set

    // App-owned external files directory for KMZ output
    val kmzDir: String by lazy {
        val ctx = ContextUtil.getContext()
        val base = ctx.getExternalFilesDir(null)
        val dir = File(base, "kmz").apply { mkdirs() }
        dir.absolutePath + File.separator
    }

    fun generateTrajectoryName(): String {
        val dateFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault())
        return "trajectory_${dateFormat.format(Date())}"
    }

    /**
     * Create a waypoint model from lat/lon/height with gimbal pitch set to -90 (looking down)
     */
    fun createWaypointFromLatLon(
        lat: Double,
        lon: Double,
        heightMeters: Double,
        index: Int
    ): WaypointInfoModel {
        val waypointInfo = WaypointInfoModel()
        val waypoint = WaylineWaypoint()

        val coordinate2D = WaylineLocationCoordinate2D().apply {
            latitude = lat
            longitude = lon
        }
        waypoint.location = coordinate2D
        waypoint.waypointIndex = index
        waypoint.height = heightMeters
        waypoint.ellipsoidHeight = heightMeters
        waypoint.useGlobalFlightHeight = false

        waypoint.useGlobalAutoFlightSpeed = true
        waypoint.useGlobalTurnParam = true

        val yawParam = WaylineWaypointYawParam().apply {
            yawMode = WaylineWaypointYawMode.FOLLOW_WAYLINE
            yawPathMode = WaylineWaypointYawPathMode.FOLLOW_BAD_ARC
            poiLocation = WaylineLocationCoordinate3D(lat, lon, heightMeters)
        }
        waypoint.yawParam = yawParam
        waypoint.useGlobalYawParam = false
        waypoint.isWaylineWaypointYawParamSet = true

        // Set gimbal pitch angle directly on the waypoint
        waypoint.gimbalPitchAngle = -90.0  // Gimbal looking straight down during trajectory following

        // Use global gimbal heading param (set at template level)
        waypoint.useGlobalGimbalHeadingParam = true

        waypointInfo.waylineWaypoint = waypoint

        // Create gimbal rotate action to set pitch to -90 degrees (looking straight down)
        val gimbalRotateParam = ActionGimbalRotateParam()
        gimbalRotateParam.enablePitch = true
        gimbalRotateParam.pitch = -90.0  // Look straight down
        gimbalRotateParam.rotateMode = WaylineGimbalActuatorRotateMode.ABSOLUTE_ANGLE
        gimbalRotateParam.payloadPositionIndex = 0
        
        val gimbalAction = WaylineActionInfo()
        gimbalAction.actionType = WaylineActionType.GIMBAL_ROTATE
        gimbalAction.gimbalRotateParam = gimbalRotateParam
        
        waypointInfo.actionInfos = arrayListOf(gimbalAction)
        return waypointInfo
    }

    fun createWaylineMission(): WaylineMission {
        val m = WaylineMission()
        val now = System.currentTimeMillis().toDouble()
        m.createTime = now
        m.updateTime = now
        return m
    }

    fun createMissionConfig(
        finishAction: WaylineFinishedAction = WaylineFinishedAction.NO_ACTION,
        lostAction: WaylineExitOnRCLostAction = WaylineExitOnRCLostAction.GO_BACK
    ): WaylineMissionConfig {
        val c = WaylineMissionConfig()
        c.flyToWaylineMode = WaylineFlyToWaylineMode.SAFELY
        c.finishAction = finishAction
        c.droneInfo = WaylineDroneInfo()
        c.securityTakeOffHeight = 20.0
        c.isSecurityTakeOffHeightSet = true
        c.exitOnRCLostBehavior = WaylineExitOnRCLostBehavior.EXCUTE_RC_LOST_ACTION
        c.exitOnRCLostType = lostAction
        c.globalTransitionalSpeed = 10.0
        c.payloadInfo = ArrayList()
        return c
    }

    private fun createTemplateWaypointInfo(
        waypointInfoModels: List<WaypointInfoModel>
    ): WaylineTemplateWaypointInfo {
        val waypoints = waypointInfoModels.map { it.waylineWaypoint }
        val info = WaylineTemplateWaypointInfo()
        info.waypoints = waypoints
        info.actionGroups = transformActionsToGroups(waypointInfoModels)  // Build proper action groups
        info.globalFlightHeight = 100.0
        info.isGlobalFlightHeightSet = true
        info.globalTurnMode = WaylineWaypointTurnMode.TO_POINT_AND_STOP_WITH_DISCONTINUITY_CURVATURE
        info.useStraightLine = true
        info.isTemplateGlobalTurnModeSet = true

        val poi = if (waypoints.isNotEmpty()) {
            val first = waypoints.first()
            first.yawParam?.poiLocation
                ?: WaylineLocationCoordinate3D(first.location.latitude, first.location.longitude, first.height)
        } else WaylineLocationCoordinate3D(0.0, 0.0, 0.0)

        val yawParam = WaylineWaypointYawParam().apply {
            yawMode = WaylineWaypointYawMode.FOLLOW_WAYLINE
            poiLocation = poi
        }
        info.globalYawParam = yawParam
        info.isTemplateGlobalYawParamSet = true
        
        // Set global gimbal heading param to look straight down (-90 degrees pitch)
        val globalGimbalParam = WaylineWaypointGimbalHeadingParam()
        globalGimbalParam.headingMode = WaylineWaypointGimbalHeadingMode.find(0)
        globalGimbalParam.pitchAngle = -90.0  // Look straight down
        info.globalGimbalHeadingParam = globalGimbalParam
        info.isTemplateGlobalGimbalHeadingParamSet = true
        
        info.pitchMode = WaylineWaypointPitchMode.USE_POINT_SETTING  // Use point setting to apply gimbal pitch
        return info
    }

    // Transform waypoint actions into proper action groups for KMZ
    private fun transformActionsToGroups(waypointInfoModels: List<WaypointInfoModel>): ArrayList<WaylineActionGroup> {
        val actionGroups = ArrayList<WaylineActionGroup>()
        
        for (i in waypointInfoModels.indices) {
            val actionInfos = waypointInfoModels[i].actionInfos
            if (actionInfos.isNotEmpty()) {
                val actionGroup = WaylineActionGroup()
                
                // Set trigger to execute when reaching waypoint
                val trigger = WaylineActionTrigger()
                trigger.setTriggerType(WaylineActionTriggerType.REACH_POINT)
                actionGroup.setTrigger(trigger)
                
                actionGroup.groupId = actionGroups.size
                actionGroup.startIndex = i
                actionGroup.endIndex = i
                actionGroup.setActions(actionInfos)
                
                // Build action tree structure
                val nodeLists = ArrayList<WaylineActionNodeList>()
                
                // Root node
                val root = WaylineActionNodeList()
                val treeNodes = ArrayList<WaylineActionTreeNode>()
                val rootNode = WaylineActionTreeNode()
                rootNode.nodeType = WaylineActionsRelationType.SEQUENCE
                rootNode.childrenNum = actionInfos.size
                treeNodes.add(rootNode)
                root.setNodes(treeNodes)
                nodeLists.add(root)
                
                // Children nodes (one for each action)
                val children = WaylineActionNodeList()
                val childrenNodeList = ArrayList<WaylineActionTreeNode>()
                for (j in actionInfos.indices) {
                    val child = WaylineActionTreeNode()
                    child.nodeType = WaylineActionsRelationType.LEAF
                    child.actionIndex = j
                    childrenNodeList.add(child)
                }
                children.setNodes(childrenNodeList)
                nodeLists.add(children)
                
                actionGroup.setNodeLists(nodeLists)
                actionGroups.add(actionGroup)
            }
        }
        
        return actionGroups
    }

    fun createTemplate(
        waypointInfoModels: List<WaypointInfoModel>,
        trajectorySpeed: Double = 5.0
    ): Template {
        val t = Template()
        t.waypointInfo = createTemplateWaypointInfo(waypointInfoModels)

        val cp = WaylineCoordinateParam().apply {
            coordinateMode = WaylineCoordinateMode.WGS84
            positioningType = WaylinePositioningType.GPS
            isWaylinePositioningTypeSet = true
            altitudeMode = WaylineAltitudeMode.RELATIVE_TO_START_POINT
        }
        t.coordinateParam = cp
        t.useGlobalTransitionalSpeed = true
        t.autoFlightSpeed = trajectorySpeed
        t.payloadParam = ArrayList()
        return t
    }

    fun extractWaylineIdsFromKmz(kmzPath: String): ArrayList<Int> {
        val result = arrayListOf<Int>()
        runCatching {
            ZipFile(File(kmzPath)).use { zip ->
                val entry: ZipEntry? = zip.getEntry("wpmz/waylines.wpml")
                if (entry != null) {
                    val text = zip.getInputStream(entry).use { it.readBytes().toString(Charsets.UTF_8) }
                    val regex = Regex("<\\s*wpml:waylineId\\s*>\\s*([0-9]+)\\s*<\\s*/\\s*wpml:waylineId\\s*>")
                    regex.findAll(text).forEach { m ->
                        m.groupValues.getOrNull(1)?.toIntOrNull()?.let { result.add(it) }
                    }
                }
            }
        }
        return result
    }

    /**
     * Generate and save a KMZ file from waypoint models
     * Returns the path to the saved KMZ file
     */
    fun generateAndSaveKmz(
        waypointInfoModels: List<WaypointInfoModel>,
        missionName: String = generateTrajectoryName(),
        trajectorySpeed: Double = 5.0,
        finishAction: WaylineFinishedAction = WaylineFinishedAction.GO_HOME,
        lostAction: WaylineExitOnRCLostAction = WaylineExitOnRCLostAction.GO_BACK
    ): String {
        WPMZManager.getInstance().init(ContextUtil.getContext())
        
        val waylineMission = createWaylineMission()
        val missionConfig = createMissionConfig(finishAction, lostAction)
        val template = createTemplate(waypointInfoModels, trajectorySpeed)
        
        val kmzOutPath = kmzDir + missionName + ".kmz"
        WPMZManager.getInstance().generateKMZFile(kmzOutPath, waylineMission, missionConfig, template)
        
        lastMissionNameNoExt = missionName
        lastMissionKmzPath = kmzOutPath
        
        return kmzOutPath
    }

    /**
     * Push a KMZ file to the aircraft
     */
    fun pushKmzToAircraft(
        kmzPath: String,
        onProgress: ((Double) -> Unit)? = null,
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        lastMissionKmzPath = kmzPath
        lastMissionNameNoExt = File(kmzPath).nameWithoutExtension
        
        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzPath, object :
            CommonCallbacks.CompletionCallbackWithProgress<Double> {
            override fun onProgressUpdate(progress: Double) {
                onProgress?.invoke(progress)
            }
            override fun onSuccess() {
                onSuccess()
            }
            override fun onFailure(error: IDJIError) {
                onFailure(error)
            }
        })
    }

    /**
     * Start a mission that has been pushed to the aircraft
     */
    fun startMission(
        missionNameNoExt: String = lastMissionNameNoExt,
        kmzPath: String = lastMissionKmzPath,
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        if (missionNameNoExt.isEmpty()) {
            val noMissionError = object : IDJIError {
                override fun errorType() = null
                override fun errorCode() = "NO_MISSION"
                override fun description() = "No mission loaded"
                override fun isError(p0: String?) = true
                override fun innerCode() = "NO_MISSION"
                override fun hint() = "Load a mission first"
            }
            onFailure(noMissionError)
            return
        }
        
        val ids = extractWaylineIdsFromKmz(kmzPath).ifEmpty { arrayListOf(0) }
        WaypointMissionManager.getInstance().startMission(
            missionNameNoExt,
            ids,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    onSuccess()
                }
                override fun onFailure(error: IDJIError) {
                    onFailure(error)
                }
            }
        )
    }

    /**
     * Pause the current mission
     */
    fun pauseMission(
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        WaypointMissionManager.getInstance().pauseMission(object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                onSuccess()
            }
            override fun onFailure(error: IDJIError) {
                onFailure(error)
            }
        })
    }

    /**
     * Stop the current mission
     */
    fun stopMission(
        missionNameNoExt: String = lastMissionNameNoExt,
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        if (missionNameNoExt.isEmpty()) {
            pauseMission(onSuccess, onFailure)
            return
        }
        WaypointMissionManager.getInstance().stopMission(missionNameNoExt, object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                onSuccess()
            }
            override fun onFailure(error: IDJIError) {
                onFailure(error)
            }
        })
    }

    fun navigateTrajectoryNative(
        userWaypoints: List<Triple<Double, Double, Double>>,
        trajectorySpeed: Double
    ) {
        if (userWaypoints.size < 2) {
            ToastUtils.showToast("Need at least 2 waypoints")
            return
        }

        if (lastMissionNameNoExt.isNotEmpty()) {
            WaypointMissionManager.getInstance().stopMission(
                lastMissionNameNoExt,
                object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() { /* no-op */ }
                    override fun onFailure(error: IDJIError) { /* ignore */ }
                }
            )
        }

        WPMZManager.getInstance().init(ContextUtil.getContext())

        val wpModels = ArrayList<WaypointInfoModel>()
        userWaypoints.forEachIndexed { idx, t ->
            wpModels.add(createWaypointFromLatLon(t.first, t.second, t.third, idx))
        }

        val mission = createWaylineMission()
        val config = createMissionConfig()
        val template = createTemplate(wpModels, trajectorySpeed)

        val missionName = generateTrajectoryName()
        val kmzOutPath = "$kmzDir$missionName.kmz"
        WPMZManager.getInstance().generateKMZFile(kmzOutPath, mission, config, template)

        lastMissionNameNoExt = missionName
        lastMissionKmzPath = kmzOutPath

        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, object :
            CommonCallbacks.CompletionCallbackWithProgress<Double> {
            override fun onProgressUpdate(progress: Double) {
            }
            override fun onSuccess() {
                val ids = extractWaylineIdsFromKmz(kmzOutPath).ifEmpty { arrayListOf(0) }
                WaypointMissionManager.getInstance().startMission(
                    lastMissionNameNoExt,
                    ids,
                    object : CommonCallbacks.CompletionCallback {
                        override fun onSuccess() {
                            ToastUtils.showToast("Mission started: $lastMissionNameNoExt")
                        }
                        override fun onFailure(error: IDJIError) {
                            ToastUtils.showToast("Start mission failed: ${error.description()}")
                        }
                    }
                )
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("Push KMZ failed: ${error.description()}")
            }
        })
    }

    fun endMission() {
        if (lastMissionNameNoExt.isEmpty()) {
            WaypointMissionManager.getInstance().pauseMission(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { ToastUtils.showToast("Mission paused") }
                override fun onFailure(error: IDJIError) { ToastUtils.showToast("No mission to stop") }
            })
            return
        }
        WaypointMissionManager.getInstance().stopMission(
            lastMissionNameNoExt,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("Mission stopped: $lastMissionNameNoExt")
                }
                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("Stop mission failed: ${error.description()}")
                }
            }
        )
    }
}
