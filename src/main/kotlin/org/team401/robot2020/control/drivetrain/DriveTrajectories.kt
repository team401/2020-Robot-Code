package org.team401.robot2020.control.drivetrain

import org.snakeskin.measure.Degrees
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimingConstraint

object DriveTrajectories {
    val pathManager = DrivetrainSubsystem.pathManager

    val testTrajectory = pathManager.generateTrajectory(
        false,
        listOf(
            Pose2d.identity(),
            Pose2d(8.0 * 12.0, 0.0, Rotation2d.identity()),
            Pose2d(16.0 * 12.0, 3.0 * 12.0, Rotation2d.identity()),
            Pose2d(18.0 * 12.0, 3.0 * 12.0, Rotation2d.identity())
        ),
        listOf<TimingConstraint<Pose2dWithCurvature>>(CentripetalAccelerationConstraint(110.0)),
        12.0 * 12.0,
        12.0 * 12.0,
        9.0
    )
}