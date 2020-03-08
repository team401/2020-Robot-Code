package org.team401.robot2020

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.AutoManager
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Feet
import org.snakeskin.measure.Inches
import org.snakeskin.registry.RealTimeTasks
import org.snakeskin.runtime.SnakeskinRuntime
import org.snakeskin.utility.value.AsyncBoolean
import org.snakeskin.utility.value.SelectableValue
import org.team401.robot2020.auto.InfiniteRechargeAuto
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.robot.SuperstructureManager
import org.team401.robot2020.control.robot.VisionManager
import org.team401.robot2020.control.turret.TurretUpdater
import org.team401.robot2020.subsystems.*
import org.team401.robot2020.util.RebootTracker
import org.team401.taxis.diffdrive.characterization.CharacterizeDrivetrainAuto
import org.team401.taxis.diffdrive.characterization.MeasureWheelRadiusAuto
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

@Setup
fun setup() {
    SelectableValue.selected = RobotConstants.COMP_INDEX
    SnakeskinRuntime.createRealTimeExecutor(RobotConstants.rtPeriod)

    AutoManager.setAutoLoop(InfiniteRechargeAuto)
    //AutoManager.setAutoLoop(MeasureWheelRadiusAuto(DrivetrainSubsystem, 36.0.Inches))
    //AutoManager.setAutoLoop(CharacterizeDrivetrainAuto(DrivetrainSubsystem))

    //Compressor().stop()

    Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick, HumanControllers.gamePad)
    Subsystems.add(DrivetrainSubsystem)
    Subsystems.add(ClimbingSubsystem)
    Subsystems.add(BallSubsystem, ShooterSubsystem)

    RealTimeTasks.add(OdometryTracker(DrivetrainSubsystem), TurretUpdater, VisionManager)

    println("REBOOTED: ${RebootTracker.hasRebooted}")

    on (Events.DISABLED) {
        VisionManager.turretVisionOff()
        SuperstructureManager.reset()
    }

    RobotState.observeFieldToTarget(readTimestamp(), Pose2d(0.0, 226.533, Rotation2d.identity()))

    SmartDashboard.putNumber("Wall to Left Bumper Inches", 0.0)
}