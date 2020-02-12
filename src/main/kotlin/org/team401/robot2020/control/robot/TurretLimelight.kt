package org.team401.robot2020.control.robot

import edu.wpi.first.networktables.*
import org.snakeskin.dsl.readTimestamp
import org.snakeskin.logic.LowPass
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Milliseconds
import org.team401.robot2020.config.FieldGeometry
import org.team401.robot2020.config.TurretGeometry
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

    private object FrameListener: TableEntryListener {
        override fun valueChanged(table: NetworkTable, key: String, entry: NetworkTableEntry, value: NetworkTableValue, flags: Int) {
            val timestamp = readTimestamp()
            if (tv.getDouble(0.0) == 1.0) {
                val latency = tl.getDouble(0.0).Milliseconds.toSeconds()
                val frameReceivedTimestamp = timestamp - latency - constantLatency
                val targetHoriz = (-1.0 * tx.getDouble(0.0)).Degrees
                val targetVert = ty.getDouble(0.0).Degrees

                val distance = (FieldGeometry.outerPortCenterHeight - TurretGeometry.turretCameraMountingHeight).value /
                        tan((TurretGeometry.turretCameraMountingAngle + targetVert).value)

                RobotState.addCameraObservation(Rotation2d.fromDegrees(targetHoriz.value), distance, frameReceivedTimestamp)
            }
        }
    }

    @Synchronized fun hasTarget(): Boolean {
        return tv.getDouble(0.0) == 1.0
    }

    @Synchronized fun start() {
        table.addEntryListener("tl", FrameListener, EntryListenerFlags.kUpdate)
    }
}