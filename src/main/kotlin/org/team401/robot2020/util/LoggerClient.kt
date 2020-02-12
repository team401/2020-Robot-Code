package org.team401.robot2020.util

import java.awt.SystemTray
import java.awt.TrayIcon
import java.io.File
import java.net.ServerSocket
import java.net.Socket
import java.nio.ByteBuffer
import javax.imageio.ImageIO
import javax.swing.SwingUtilities

object LoggerClient {
    private val tray = SystemTray.getSystemTray()
    private val trayIcon = TrayIcon(ImageIO.read(javaClass.classLoader.getResource("robot-icon.png")), "401 Logger Client")

    init {
        trayIcon.isImageAutoSize = true
        tray.add(trayIcon)
    }

    class SessionHandler(val socket: Socket, val directory: String): Runnable {
        private val file = File("$directory/${System.currentTimeMillis()}.csv")
        private val printWriter = file.printWriter()

        override fun run() {
            val stream = socket.getInputStream()
            val bufReader = stream.bufferedReader()
            val headerStr = bufReader.readLine()
            val header = headerStr.split(", ")
            val numBytes = header.size * java.lang.Double.BYTES
            val array = ByteArray(numBytes)
            val byteBuf = ByteBuffer.wrap(array)
            val doubleBuf = byteBuf.asDoubleBuffer()

            printWriter.println(headerStr)
            printWriter.flush()

            while (socket.isConnected) {
                val resp = stream.read(array)
                if (resp == -1) break //Client disconnected

                val values = arrayListOf<Double>()
                for (i in header.indices) {
                    values.add(doubleBuf.get(i))
                }
                printWriter.println(values.joinToString())
                printWriter.flush()
            }


            trayIcon.displayMessage("Received Log File", file.path, TrayIcon.MessageType.INFO)
            socket.close()
            printWriter.close()
        }
    }

    @JvmStatic
    fun main(args: Array<String>) {
        val server = ServerSocket(args[0].toInt())
        val directory = args[1]

        while (true) {
            val client = server.accept()
            val thread = Thread(SessionHandler(client, directory))
            thread.start()
        }
    }
}