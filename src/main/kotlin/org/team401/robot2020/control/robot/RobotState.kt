package org.team401.robot2020.control.robot

import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2020.config.TurretGeometry
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.util.InterpolatingDouble
import org.team401.taxis.util.InterpolatingTreeMap

object RobotState: DifferentialDriveState(100, DrivetrainSubsystem.model.driveKinematicsModel) {
    private val robotToTurret = InterpolatingTreeMap<InterpolatingDouble, Pose2d>(100)

    @Synchronized fun addTurretObservation(turretAngle: Rotation2d, time: TimeMeasureSeconds) {
        val robotToTurretPose = Pose2d(TurretGeometry.robotToTurretTranslation, turretAngle)
        robotToTurret[InterpolatingDouble(time.value)] = robotToTurretPose
    }
}