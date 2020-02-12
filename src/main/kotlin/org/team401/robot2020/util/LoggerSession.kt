package org.team401.robot2020.util

import java.io.PrintWriter
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.DoubleBuffer

/**
 * A logger session for sending log data to a remote receiver in a CSV format.
 */
class LoggerSession(val server: String, val port: Int) {
    private val header = arrayListOf<String>()
    private val preHeaderBuffer = arrayListOf<Double>()
    private lateinit var buffer: ByteBuffer
    private lateinit var doubleBuf: DoubleBuffer
    private var isHeaderPublished = false
    private val socket = Socket(server, port)
    private val stream = socket.getOutputStream()
    private val writer = PrintWriter(stream, true, Charsets.UTF_8)

    /**
     * Sets a value in the current log session
     */
    operator fun set(name: String, value: Double) {
        if (!isHeaderPublished) {
            header.add(name)
            preHeaderBuffer.add(value)
        } else {
            val idx = header.indexOf(name)
            doubleBuf.put(idx, value)
        }
    }

    /**
     * Sends the current buffer data to the server.  This should be called at the end of all set calls.
     */
    fun publish() {
        if (!isHeaderPublished) {
            //Need to publish the header
            writer.println(header.joinToString())
            buffer = ByteBuffer.allocate(java.lang.Double.BYTES * header.size)
            doubleBuf = buffer.asDoubleBuffer()

            preHeaderBuffer.forEachIndexed { index, d ->
                doubleBuf.put(index, d)
            }
            preHeaderBuffer.clear()

            isHeaderPublished = true
        }

        //Write data to the server
        stream.write(buffer.array())
    }

    /**
     * Ends the session, telling the server to save the received data to file.
     */
    fun end() {
        socket.close()
    }
}

fun main() {
    val session = LoggerSession("localhost", 5801)

    var timestamp = 0.0

    for (i in 0 until 1000) {
        session["Timestamp (s)"] = timestamp
        session["Test value 1"] = Math.random()
        session["Test value 2"] = Math.random()
        session.publish()
        timestamp += .01
    }

    session.end()
}