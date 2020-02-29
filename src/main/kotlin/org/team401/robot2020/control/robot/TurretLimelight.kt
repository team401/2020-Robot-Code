package org.team401.robot2020.control.robot

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.LinearFilter
import org.snakeskin.dsl.readTimestamp
import org.snakeskin.logic.LowPass
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Milliseconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2020.config.FieldGeometry
import org.team401.taxis.geometry.Rotation2d
import kotlin.math.floor
import kotlin.math.tan

object TurretLimelight {
    private val constantLatency = 11.0.Milliseconds.toSeconds()

    private val table = NetworkTableInstance.getDefault().getTable("limelight-turret")

    val tx = table.getEntry("tx")
    private val ty = table.getEntry("ty")
    private val tl = table.getEntry("tl")
    private val tv = table.getEntry("tv")

    private val ledMode = table.getEntry("ledMode")

    //private val txFilter by lazy { LinearFilter.movingAverage(5) }

    fun captureFrame(timestamp: TimeMeasureSeconds): Boolean {
        //Retrieve a frame from the camera
        return if (tv.getDouble(0.0) == 1.0) {
            val latency = tl.getDouble(0.0).Milliseconds.toSeconds()
            val frameReceivedTimestamp = timestamp - latency - constantLatency
            val targetHoriz = (-1.0 * tx.getDouble(0.0)).Degrees
            val targetVert = ty.getDouble(0.0).Degrees

            RobotState.addCameraObservation(Rotation2d.fromDegrees(targetHoriz.value), 0.0, frameReceivedTimestamp)
            true
        } else {
            false
        }
    }

    @Synchronized fun hasTarget(): Boolean {
        return tv.getDouble(0.0) == 1.0
    }

    fun ledOn() {
        ledMode.setDouble(0.0)
    }

    fun ledOff() {
        ledMode.setDouble(1.0)
    }
}