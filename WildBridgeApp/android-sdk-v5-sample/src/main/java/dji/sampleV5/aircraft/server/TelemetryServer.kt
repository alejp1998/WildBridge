package dji.sampleV5.aircraft.server

import android.util.Log
import java.io.PrintWriter
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.Executors
import kotlin.concurrent.thread

class TelemetryServer(
    private val port: Int,
    private val telemetryProvider: () -> String
) {
    private var serverSocket: ServerSocket? = null
    private val executor = Executors.newCachedThreadPool()
    @Volatile
    private var isRunning = false
    private var serverThread: Thread? = null
    private val clients = ConcurrentHashMap<Socket, PrintWriter>()

    /** Callback invoked when a bridge client connects. Receives the client's IP address. */
    var onFirstClientConnected: ((clientIp: String) -> Unit)? = null

    fun hasClients(): Boolean = clients.isNotEmpty()

    fun start() {
        if (isRunning) return

        serverThread = thread(name = "TelemetryServer-$port", start = true) {
            runCatching {
                serverSocket = ServerSocket(port)
                isRunning = true
                Log.i("TelemetryServer", "Server started on port $port")

                // Start a thread to periodically send telemetry data to all connected clients
                executor.submit { sendTelemetryData() }

                while (isRunning && !serverSocket!!.isClosed) {
                    acceptNextClient()
                }
            }.onFailure { error ->
                Log.e("TelemetryServer", "Server error: ${error.message}")
            }
        }
    }

    private fun acceptNextClient() {
        runCatching {
            val clientSocket = serverSocket!!.accept()
            val clientIp = clientSocket.inetAddress.hostAddress ?: "unknown"
            Log.i("TelemetryServer", "Client connected: $clientIp")
            val writer = PrintWriter(clientSocket.getOutputStream(), true)
            clients[clientSocket] = writer
            onFirstClientConnected?.invoke(clientIp)
        }.onFailure { error ->
            if (isRunning) {
                Log.e("TelemetryServer", "Error accepting connection: ${error.message}")
            }
        }
    }

    private fun sendTelemetryData() {
        while (isRunning) {
            runCatching {
                val telemetryJson = telemetryProvider()
                removeDisconnectedClients(sendTelemetryToClients(telemetryJson))
                Thread.sleep(10) // Send data at ~100Hz (cache rebuilt by SDK listeners)
            }.onFailure { error ->
                handleTelemetryLoopError(error)
            }
        }
    }

    private fun sendTelemetryToClients(telemetryJson: String): List<Socket> {
        return clients.mapNotNull { (socket, writer) ->
            if (socket.isClosed || !socket.isConnected) {
                socket
            } else {
                writer.println(telemetryJson)
                socket.takeIf { writer.checkError() }
            }
        }
    }

    private fun removeDisconnectedClients(clientsToRemove: List<Socket>) {
        clientsToRemove.forEach { socket ->
            runCatching { socket.close() }
                .onFailure { error -> Log.d("TelemetryServer", "Socket close ignored: ${error.message}") }
            clients.remove(socket)
            Log.i("TelemetryServer", "Client disconnected and removed.")
        }
    }

    private fun handleTelemetryLoopError(error: Throwable) {
        if (error is InterruptedException) {
            Thread.currentThread().interrupt()
            isRunning = false
        } else {
            Log.e("TelemetryServer", "Error in telemetry sending loop: ${error.message}")
        }
    }

    fun stop() {
        isRunning = false
        onFirstClientConnected = null
        runCatching {
            clients.values.forEach { it.close() }
            clients.keys.forEach { it.close() }
            clients.clear()
            serverSocket?.close()
            serverSocket = null
            executor.shutdownNow()
            if (Thread.currentThread() != serverThread) {
                serverThread?.join(1000)
            }
            serverThread = null
        }.onFailure { error -> Log.e("TelemetryServer", "Error stopping server: ${error.message}") }
    }
}

