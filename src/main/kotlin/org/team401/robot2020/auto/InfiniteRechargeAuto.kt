package org.team401.robot2020.auto

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.*
import org.team401.robot2020.auto.steps.DriveTrajectoryStep
import org.team401.robot2020.control.drivetrain.DriveTrajectories

object InfiniteRechargeAuto: RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return auto {
            step(DriveTrajectoryStep(DriveTrajectories.testTrajectory, true))
        }
    }
}