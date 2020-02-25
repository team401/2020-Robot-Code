package org.team401.robot2020

import org.snakeskin.auto.AutoManager
import org.snakeskin.dsl.*
import org.snakeskin.registry.RealTimeTasks
import org.snakeskin.runtime.SnakeskinRuntime
import org.snakeskin.utility.value.SelectableValue
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.control.robot.TurretLimelight
import org.team401.robot2020.control.turret.TurretUpdater
import org.team401.robot2020.subsystems.*
import org.team401.taxis.diffdrive.odometry.OdometryTracker

@Setup
fun setup() {
    SelectableValue.selected = RobotConstants.PRACTICE_INDEX
    SnakeskinRuntime.createRealTimeExecutor(RobotConstants.rtPeriod)

    Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick, HumanControllers.gamePad)
    Subsystems.add(DrivetrainSubsystem, BallSubsystem, ClimbingSubsystem, ShooterSubsystem)

    RealTimeTasks.add(OdometryTracker(DrivetrainSubsystem), TurretUpdater)
    TurretLimelight.start()
    TurretLimelight.ledOff()
}