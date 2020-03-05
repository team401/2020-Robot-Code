package org.team401.robot2020

import org.snakeskin.auto.AutoManager
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.registry.RealTimeTasks
import org.snakeskin.runtime.SnakeskinRuntime
import org.snakeskin.utility.value.AsyncBoolean
import org.snakeskin.utility.value.SelectableValue
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.control.robot.SuperstructureManager
import org.team401.robot2020.control.robot.VisionManager
import org.team401.robot2020.control.turret.TurretUpdater
import org.team401.robot2020.subsystems.*
import org.team401.taxis.diffdrive.odometry.OdometryTracker

@Setup
fun setup() {
    SelectableValue.selected = RobotConstants.COMP_INDEX
    SnakeskinRuntime.createRealTimeExecutor(RobotConstants.rtPeriod)

    AutoManager.setAutoLoop(ShooterSubsystem.flywheelCharacterizationRoutine.loop)

    Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick, HumanControllers.gamePad)
    Subsystems.add(DrivetrainSubsystem, BallSubsystem, ShooterSubsystem)

    RealTimeTasks.add(OdometryTracker(DrivetrainSubsystem), TurretUpdater, VisionManager)

    on (Events.DISABLED) {
        VisionManager.turretVisionOff()
        SuperstructureManager.reset()
    }
}