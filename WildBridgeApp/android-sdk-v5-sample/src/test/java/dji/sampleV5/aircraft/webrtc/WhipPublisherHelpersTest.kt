package dji.sampleV5.aircraft.webrtc

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.net.URL

class WhipPublisherHelpersTest {
    @Test
    fun absoluteWhipResourceUrlKeepsAbsoluteLocation() {
        val result = absoluteWhipResourceUrl(
            URL("http://media.local:8889/drone/whip"),
            "https://example.test/session/123"
        )

        assertEquals("https://example.test/session/123", result)
    }

    @Test
    fun absoluteWhipResourceUrlResolvesRelativeLocationWithExplicitPort() {
        val result = absoluteWhipResourceUrl(
            URL("http://media.local:8889/drone/whip"),
            "/session/123"
        )

        assertEquals("http://media.local:8889/session/123", result)
    }

    @Test
    fun absoluteWhipResourceUrlResolvesRelativeLocationWithoutDefaultPortSentinel() {
        val result = absoluteWhipResourceUrl(
            URL("https://media.local/drone/whip"),
            "/session/123"
        )

        assertEquals("https://media.local/session/123", result)
    }

    @Test
    fun absoluteWhipResourceUrlReturnsNullWhenLocationMissing() {
        assertNull(absoluteWhipResourceUrl(URL("http://media.local:8889/drone/whip"), null))
    }

    @Test
    fun firstFrameGateReturnsWhenFirstWaitSucceeds() {
        val waiter = FakeFirstFrameWaiter(totalFrames = 12L, waitResults = listOf(true))
        val gate = WhipFirstFrameGate(waiter, unavailableMessage = "missing frame")

        gate.awaitFirstFrame(startingFrameCount = 12L, firstTimeoutMs = 8_000L, recoveryTimeoutMs = 4_000L)

        assertEquals(12L, gate.totalOutputFrames())
        assertEquals(listOf(8_000L), waiter.requestedTimeouts)
    }

    @Test
    fun firstFrameGateRecoversAndRetriesWhenConfigured() {
        val waiter = FakeFirstFrameWaiter(totalFrames = 2L, waitResults = listOf(false, true))
        var recovered = false
        val gate = WhipFirstFrameGate(
            waiter = waiter,
            unavailableMessage = "missing dji frame",
            recoverBeforeRetry = { recovered = true }
        )

        gate.awaitFirstFrame(startingFrameCount = 2L, firstTimeoutMs = 8_000L, recoveryTimeoutMs = 4_000L)

        assertTrue(recovered)
        assertEquals(listOf(8_000L, 4_000L), waiter.requestedTimeouts)
    }

    @Test
    fun firstFrameGateFailsWithoutRecoveryWhenFirstWaitFails() {
        val waiter = FakeFirstFrameWaiter(totalFrames = 0L, waitResults = listOf(false))
        val gate = WhipFirstFrameGate(waiter, unavailableMessage = "missing mock frame")

        val failure = runCatching {
            gate.awaitFirstFrame(startingFrameCount = 0L, firstTimeoutMs = 8_000L, recoveryTimeoutMs = 4_000L)
        }.exceptionOrNull()

        assertEquals("missing mock frame", failure?.message)
        assertFalse(waiter.requestedTimeouts.contains(4_000L))
    }

    @Test
    fun firstFrameGateFailsAfterRecoveryRetryWhenSecondWaitFails() {
        val waiter = FakeFirstFrameWaiter(totalFrames = 0L, waitResults = listOf(false, false))
        var recoveryCount = 0
        val gate = WhipFirstFrameGate(
            waiter = waiter,
            unavailableMessage = "missing dji frame",
            recoverBeforeRetry = { recoveryCount++ }
        )

        val failure = runCatching {
            gate.awaitFirstFrame(startingFrameCount = 0L, firstTimeoutMs = 8_000L, recoveryTimeoutMs = 4_000L)
        }.exceptionOrNull()

        assertEquals("missing dji frame", failure?.message)
        assertEquals(1, recoveryCount)
        assertEquals(listOf(8_000L, 4_000L), waiter.requestedTimeouts)
    }

    private class FakeFirstFrameWaiter(
        private val totalFrames: Long,
        waitResults: List<Boolean>
    ) : WhipFirstFrameWaiter {
        private val results = ArrayDeque(waitResults)
        val requestedTimeouts = mutableListOf<Long>()

        override fun totalOutputFrames(): Long = totalFrames

        override fun waitForOutputFrameAfter(frameCount: Long, timeoutMs: Long): Boolean {
            requestedTimeouts += timeoutMs
            return results.removeFirstOrNull() ?: false
        }
    }
}