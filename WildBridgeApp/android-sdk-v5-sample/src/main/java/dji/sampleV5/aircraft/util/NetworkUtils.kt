package dji.sampleV5.aircraft.util

import android.util.Log
import java.io.IOException
import java.net.Inet4Address
import java.net.NetworkInterface
import java.net.ServerSocket
import java.net.SocketException
import java.util.Collections

/**
 * A structure representing an IPv4 candidate found on a network interface.
 */
data class DeviceIpCandidate(
    val ip: String,
    val interfaceName: String
) {
    val isPreferred: Boolean
        get() = interfaceName.contains("wlan") || interfaceName.contains("ap")
}

/**
 * Clean utilities for scanning network interfaces and checking port availability.
 * Follows strict exception hygiene (naming ignored catches 'ignored').
 */
object NetworkUtils {
    private const val TAG = "NetworkUtils"

    /**
     * Resolves the primary device IP address, preferring Wi-Fi/Access Point interfaces.
     */
    fun getDeviceIpAddress(): String? {
        return try {
            val candidates = deviceIpCandidates()
            candidates.forEach {
                Log.d(TAG, "Found IP: ${it.ip} on interface: ${it.interfaceName}")
            }
            (candidates.firstOrNull { it.isPreferred } ?: candidates.firstOrNull())?.ip
        } catch (ignored: SocketException) {
            null
        }
    }

    /**
     * Scans and returns all active IPv4 candidates across network interfaces.
     */
    fun deviceIpCandidates(): List<DeviceIpCandidate> {
        return try {
            Collections.list(NetworkInterface.getNetworkInterfaces())
                .filter { it.isUsableNetworkInterface() }
                .flatMap { it.ipv4Candidates() }
        } catch (ignored: SocketException) {
            emptyList()
        }
    }

    /**
     * Helper to verify if a local port is already in use by another socket.
     */
    fun isPortInUse(port: Int): Boolean {
        return try {
            ServerSocket(port).close()
            false
        } catch (ignored: IOException) {
            true
        }
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
}
