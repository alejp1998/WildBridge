package dji.sampleV5.aircraft.detection

import java.util.concurrent.atomic.AtomicReference

/**
 * Singleton that holds the most recent detection results.
 *
 * - Written by RhinoYoloDetector (inference thread)
 * - Read by DetectionOverlayView (main thread, onDraw) and
 *   TelemetryProvider (camera frame thread, for WebRTC metadata)
 *
 * Results older than [MAX_RESULT_AGE_MS] are treated as stale and
 * [getFreshDetections] returns an empty list instead of outdated boxes.
 */
object DetectionState {

    const val MAX_RESULT_AGE_MS = 2500L

    private data class Snapshot(
        val results: List<DetectionResult>,
        val timestampMs: Long
    )

    private val _snapshot = AtomicReference(Snapshot(emptyList(), 0L))

    /** Called from the inference thread after each successful detection run. */
    fun update(results: List<DetectionResult>) {
        _snapshot.set(Snapshot(results, System.currentTimeMillis()))
    }

    /**
     * Returns the latest results if they are younger than [MAX_RESULT_AGE_MS],
     * otherwise returns an empty list (stale-result protection).
     */
    fun getFreshDetections(): List<DetectionResult> {
        val s = _snapshot.get()
        return if (System.currentTimeMillis() - s.timestampMs < MAX_RESULT_AGE_MS) {
            s.results
        } else {
            emptyList()
        }
    }

    /** Raw snapshot including timestamp – useful for overlay age-check. */
    val latestTimestampMs: Long get() = _snapshot.get().timestampMs

    /** Clear all stored detections immediately (e.g., when detection is toggled OFF). */
    fun clear() = update(emptyList())
}
