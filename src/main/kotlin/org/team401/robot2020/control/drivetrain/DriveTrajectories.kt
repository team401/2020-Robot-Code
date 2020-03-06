package org.team401.robot2020.control.drivetrain

import org.team401.robot2020.config.FieldGeometry
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.visualization.visualize
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimedState
import org.team401.taxis.trajectory.timing.TimingConstraint
import org.team401.taxis.trajectory.timing.VelocityLimitRegionConstraint

object DriveTrajectories {
    private const val startingLineDistance = 120.938
    private val pathManager = DrivetrainSubsystem.pathManager

    private val centripetalConstraint = CentripetalAccelerationConstraint(120.0)
    private val trenchVelocityConstraint = VelocityLimitRegionConstraint<Pose2dWithCurvature>(
        Translation2d(206.557, 265.689),
        Translation2d(422.567, 321.189),
        6.0 * 12.0
    )

    object Trench6Trajectories {
        private val enterTrenchPose = Pose2d(242.441, 293.439, Rotation2d.identity()).transformBy(RobotConstants.intakeToRobotOrigin)
        private val endPose = enterTrenchPose.transformBy(Pose2d.fromTranslation(Translation2d(76.0, 0.0)))

        private val returnMidPose = Pose2d(206.0, 240.0, Rotation2d.fromDegrees(21.5))
        private val shootPose = Pose2d(140.0, 226.533, Rotation2d.identity())

        fun generateTrench6TrajectoryCollect(wallToRightBumper: Double): Trajectory<TimedState<Pose2dWithCurvature>> {
            val startingPose = Pose2d.fromTranslation(Translation2d(startingLineDistance, wallToRightBumper))
                .transformBy(RobotConstants.bumperRightToOrigin)

            return pathManager.generateTrajectory(
                false,
                listOf(
                    startingPose,
                    enterTrenchPose,
                    endPose
                ),
                listOf<TimingConstraint<Pose2dWithCurvature>>(centripetalConstraint, trenchVelocityConstraint),
                12.0 * 12.0,
                24.0 * 12.0,
                9.0
            )
        }

       val trench6TrajectoryReturn = pathManager.generateTrajectory(
           true,
           listOf(
               endPose,
               returnMidPose,
               shootPose
           ),
           listOf<TimingConstraint<Pose2dWithCurvature>>(centripetalConstraint),
           12.0 * 12.0,
           24.0 * 12.0,
           9.0
       )
    }

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
    DriveTrajectories.Trench6Trajectories.trench6TrajectoryReturn.visualize(DrivetrainSubsystem, FieldGeometry.fieldLines)
}