package org.team401.robot2020

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.AutoManager
import org.snakeskin.dsl.*
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.registry.RealTimeTasks
import org.snakeskin.rt.RealTimeTask
import org.snakeskin.runtime.SnakeskinRuntime
import org.snakeskin.utility.value.AsyncBoolean
import org.snakeskin.utility.value.SelectableValue
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.robot.SuperstructureManager
import org.team401.robot2020.control.robot.TurretLimelight
import org.team401.robot2020.subsystems.*
import org.team401.sevision.LimelightCamera
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d

var visionEnabled by AsyncBoolean(false)

@Setup
fun setup() {
    SelectableValue.selected = RobotConstants.COMP_INDEX
    SnakeskinRuntime.createRealTimeExecutor(RobotConstants.rtPeriod)

    AutoManager.setAutoLoop(ShooterSubsystem.flywheelCharacterizationRoutine.loop)

    Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick, HumanControllers.gamePad)
    Subsystems.add(DrivetrainSubsystem, BallSubsystem, ShooterSubsystem)

    RealTimeTasks.add(OdometryTracker(DrivetrainSubsystem))
    //TurretLimelight.ledOff()

    val limelight = LimelightCamera("turret", Pose2d(-2.625, 0.0, Rotation2d.fromDegrees(-2.0)), 42.25, Rotation2d.fromDegrees(28.9))

    val updaterTask = object : RealTimeTask() {
        override fun action(timestamp: TimeMeasureSeconds, dt: TimeMeasureSeconds) {
            if (!visionEnabled) {
                RobotState.addVisionUpdate(
                    timestamp - limelight.latency.Seconds,
                    null,
                    limelight
                )
                return
            }
            RobotState.addVisionUpdate(
                timestamp - limelight.latency.Seconds,
                limelight.target,
                limelight
            )
        }
    }

    RealTimeTasks.add(updaterTask, SuperstructureManager)

    SmartDashboard.putNumber("camera_angle", 0.0)

    println("SHOOTER R^2: ${ShooterConstants.flywheelRegression.R2()}")
}