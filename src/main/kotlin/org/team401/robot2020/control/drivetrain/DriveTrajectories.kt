package org.team401.robot2020.control.drivetrain

import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.visualization.visualize
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.timing.TimingConstraint

object DriveTrajectories {
    val pathManager = DrivetrainSubsystem.pathManager

    val testTrajectory = pathManager.generateTrajectory(
        false,
        listOf(
            Pose2d(24.0, 240.0, Rotation2d.fromDegrees(90.0)),
            Pose2d(24.0 + 120.0, 240.0 + 120.0, Rotation2d.fromDegrees(0.0)),
            Pose2d(24.0 + 120.0 + 120.0, 240.0, Rotation2d.fromDegrees(-90.0)),
            Pose2d(24.0 + 120.0, 240.0 - 120.0, Rotation2d.fromDegrees(-180.0)),
            Pose2d(24.0, 240.0, Rotation2d.fromDegrees(-270.0))
        ),
        listOf<TimingConstraint<Pose2dWithCurvature>>(),
        12.0 * 12.0,
        12.0 * 12.0,
        9.0
    )
}

fun main() {
    DriveTrajectories.testTrajectory.visualize(DrivetrainSubsystem)
}