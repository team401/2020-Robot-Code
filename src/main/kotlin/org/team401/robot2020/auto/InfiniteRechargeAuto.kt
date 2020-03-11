package org.team401.robot2020.auto

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.*
import org.snakeskin.measure.Seconds
import org.team401.robot2020.auto.steps.DriveTrajectoryStep
import org.team401.robot2020.auto.steps.OpenLoopDriveStep
import org.team401.robot2020.auto.steps.WaitForOdometry
import org.team401.robot2020.control.drivetrain.DriveTrajectories
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.robot.SuperstructureManager
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

object InfiniteRechargeAuto: RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return auto {
            step { RobotState.resetVision() }
            step(OpenLoopDriveStep(0.2, 1.0.Seconds))
            step { SuperstructureManager.lockNearShot() }
            delay(0.5.Seconds)
            step { SuperstructureManager.startFiring() }
            delay(6.0.Seconds)
            step { SuperstructureManager.unwindShooter() }
        }
    }
}