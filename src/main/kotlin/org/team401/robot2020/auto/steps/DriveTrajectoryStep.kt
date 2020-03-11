package org.team401.robot2020.auto.steps

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.TimedState

class DriveTrajectoryStep(referenceTrajectory: Trajectory<TimedState<Pose2dWithCurvature>>, val reconfigurePose: Boolean = false): AutoStep() {
    private val trajectory = TrajectoryIterator(TimedView(referenceTrajectory))

    override fun entry(currentTime: TimeMeasureSeconds) {
        if (reconfigurePose) { //If we're supposed to reconfigure the pose
            val startPose = trajectory.state.state().pose //Grab the initial pose
            DrivetrainSubsystem.setPose(startPose, currentTime) //Reconfigure the drive
        }
        DrivetrainSubsystem.pathManager.reset() //Reset the path manager
        DrivetrainSubsystem.pathManager.setTrajectory(trajectory) //Load in the path
        DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.TrajectoryFollowing).waitFor()
    }

    override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds): Boolean {
        return DrivetrainSubsystem.pathManager.isDone
    }

    override fun exit(currentTime: TimeMeasureSeconds) {
        DrivetrainSubsystem.driveMachine.disable()
    }
}