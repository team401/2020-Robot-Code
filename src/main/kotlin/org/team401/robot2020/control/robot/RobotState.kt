package org.team401.robot2020.control.robot

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.runtime.SnakeskinPlatform
import org.snakeskin.runtime.SnakeskinRuntime
import org.team401.robot2020.config.TurretGeometry
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.util.InterpolatingDouble
import org.team401.taxis.util.InterpolatingTreeMap

object RobotState: DifferentialDriveState(100, DrivetrainSubsystem.model.driveKinematicsModel) {
    private val vehicleToTurret = InterpolatingTreeMap<InterpolatingDouble, Rotation2d>(observationBufferSize)
    private val cameraToGoal = InterpolatingTreeMap<InterpolatingDouble, Pose2d>(observationBufferSize)

    init {
        cameraToGoal[InterpolatingDouble(0.0)] = Pose2d.identity()
    }

    @Synchronized fun addTurretObservation(turretAngle: Rotation2d, time: TimeMeasureSeconds) {
        vehicleToTurret[InterpolatingDouble(time.value)] = turretAngle
    }

    @Synchronized fun addCameraObservation(angleToGoal: Rotation2d, distanceToGoal: Double, time: TimeMeasureSeconds) {
        cameraToGoal[InterpolatingDouble(time.value)] = Pose2d(distanceToGoal, 0.0, angleToGoal)
    }

    @Synchronized fun getVehicleToTurret(timestamp: TimeMeasureSeconds): Rotation2d {
        return vehicleToTurret.getInterpolated(InterpolatingDouble(timestamp.value))
    }

    @Synchronized fun getTurretAimingParameters(timestamp: TimeMeasureSeconds): TurretAimingParameters {
        val latestCameraToGoalEntry = cameraToGoal.lastEntry()
        val latestCameraToGoal = latestCameraToGoalEntry.value
        val latestCameraTimestamp = latestCameraToGoalEntry.key.value.Seconds

        val fieldToVehicleAtFrame = getFieldToVehicle(latestCameraTimestamp)
        val fieldToVehicleNow = getFieldToVehicle(timestamp)
        val vehicleToTurretAtFrame = getVehicleToTurret(latestCameraTimestamp)
        val vehicleToTurretNow = getVehicleToTurret(timestamp)

        val fieldToTurretAtFrame = fieldToVehicleAtFrame.rotation.rotateBy(vehicleToTurretAtFrame)
        val fieldToTurretNow = fieldToVehicleNow.rotation.rotateBy(vehicleToTurretNow)

        val turretDelta = fieldToTurretNow.rotateBy(fieldToTurretAtFrame.inverse())
        val correctedAngle = latestCameraToGoal.rotation.rotateBy(turretDelta.inverse())
        val desiredTurretAngle = vehicleToTurretNow.rotateBy(correctedAngle)

        return TurretAimingParameters(desiredTurretAngle, 0.0.Inches)
    }
}