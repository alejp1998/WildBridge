package dji.sampleV5.aircraft.webrtc

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
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
}