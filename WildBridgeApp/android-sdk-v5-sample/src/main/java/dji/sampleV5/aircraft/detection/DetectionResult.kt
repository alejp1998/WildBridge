package dji.sampleV5.aircraft.detection

import org.json.JSONArray
import org.json.JSONObject

/**
 * A single object detection result from RhinoYoloDetector.
 *
 * @param classId      Integer class index in the label list
 * @param className    Human-readable class name (e.g. "rhino")
 * @param confidence   Confidence in [0, 1]
 * @param x1           Normalised left   (relative to frame width,  0-1)
 * @param y1           Normalised top    (relative to frame height, 0-1)
 * @param x2           Normalised right  (relative to frame width,  0-1)
 * @param y2           Normalised bottom (relative to frame height, 0-1)
 * @param inferenceMs  Time the inference run took in milliseconds
 * @param modelName    Identifier of the model that produced this result
 * @param frameNumber  Frame counter this detection belongs to
 */
data class DetectionResult(
    val classId: Int,
    val className: String,
    val confidence: Float,
    // Normalised bounding box [0, 1]
    val x1: Float,
    val y1: Float,
    val x2: Float,
    val y2: Float,
    val inferenceMs: Long = 0L,
    val modelName: String = "rhino_yolo26s",
    val frameNumber: Long = 0L
) {
    fun toJson(): JSONObject = JSONObject().apply {
        put("classId", classId)
        put("className", className)
        put("confidence", confidence.toDouble())
        put("bbox", JSONArray().apply {
            put(x1.toDouble()); put(y1.toDouble())
            put(x2.toDouble()); put(y2.toDouble())
        })
        put("inferenceMs", inferenceMs)
        put("modelName", modelName)
        put("frameNumber", frameNumber)
    }

    companion object {
        fun listToJsonArray(results: List<DetectionResult>): JSONArray =
            JSONArray().also { arr -> results.forEach { arr.put(it.toJson()) } }

        fun listFromJsonArray(array: JSONArray): List<DetectionResult> =
            (0 until array.length()).map { i ->
                val obj = array.getJSONObject(i)
                val bbox = obj.optJSONArray("bbox") ?: JSONArray()
                DetectionResult(
                    classId    = obj.optInt("classId", 0),
                    className  = obj.optString("className", "unknown"),
                    confidence = obj.optDouble("confidence", 0.0).toFloat(),
                    x1 = bbox.optDouble(0, 0.0).toFloat(),
                    y1 = bbox.optDouble(1, 0.0).toFloat(),
                    x2 = bbox.optDouble(2, 0.0).toFloat(),
                    y2 = bbox.optDouble(3, 0.0).toFloat(),
                    inferenceMs  = obj.optLong("inferenceMs", 0),
                    modelName    = obj.optString("modelName", "rhino_yolo26s"),
                    frameNumber  = obj.optLong("frameNumber", 0)
                )
            }
    }
}
