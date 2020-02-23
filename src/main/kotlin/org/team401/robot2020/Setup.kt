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
    SelectableValue.selected = RobotConstants.COMP_INDEX

    SnakeskinRuntime.createRealTimeExecutor(RobotConstants.rtPeriod)
    //AutoManager.setAutoLoop(BallSubsystem.armCharacterizationRoutine.loop)
    //AutoManager.setAutoLoop(ClimbingSubsystem.rightElevatorCharacterizationRoutine.loop)
    //AutoManager.setAutoLoop(ShooterSubsystem.turretCharacterizationRoutine.loop)
    //AutoManager.setAutoLoop(ShooterSubsystem.shooterCharacterizationRoutine.loop)
    //Controllers.add(HumanControllers.gamePad)
    Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick)
    Subsystems.add(DrivetrainSubsystem, ShooterSubsystem)
    //Subsystems.add(BallSubsystem)
    RealTimeTasks.add(OdometryTracker(DrivetrainSubsystem))
    RealTimeTasks.add(TurretUpdater)
    TurretLimelight.start()
}