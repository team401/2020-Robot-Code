package org.team401.robot2020.control.robot

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.LinearFilter
import org.snakeskin.dsl.readTimestamp
import org.snakeskin.logic.LowPass
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Milliseconds
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

    private val txFilter by lazy { LinearFilter.movingAverage(5) }

    private object FrameListener: TableEntryListener {
        override fun valueChanged(table: NetworkTable, key: String, entry: NetworkTableEntry, value: NetworkTableValue, flags: Int) {
            val timestamp = readTimestamp()
            if (tv.getDouble(0.0) == 1.0) {
                val latency = tl.getDouble(0.0).Milliseconds.toSeconds()
                val frameReceivedTimestamp = timestamp - latency - constantLatency
                val targetHoriz = txFilter.calculate((-1.0 * tx.getDouble(0.0))).Degrees
                val targetVert = ty.getDouble(0.0).Degrees

                /* TODO add distance calculation (requires mode detection of far vs. near)
                val distance = (FieldGeometry.outerPortCenterHeight - TurretGeometry.turretCameraMountingHeight).value /
                        tan((TurretGeometry.turretCameraMountingAngle + targetVert).value)
                 */

                RobotState.addCameraObservation(Rotation2d.fromDegrees(targetHoriz.value), 0.0, frameReceivedTimestamp)
            } else {
                txFilter.reset()
            }
        }
    }

    @Synchronized fun hasTarget(): Boolean {
        return tv.getDouble(0.0) == 1.0
    }

    @Synchronized fun start() {
        table.addEntryListener("tl", FrameListener, EntryListenerFlags.kUpdate)
    }

    fun ledOn() {
        ledMode.setDouble(0.0)
    }

    fun ledOff() {
        ledMode.setDouble(1.0)
    }
}