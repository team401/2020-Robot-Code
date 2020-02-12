package org.team401.robot2020

import org.snakeskin.auto.AutoManager
import org.snakeskin.dsl.*
import org.snakeskin.init.InitManager
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Seconds
import org.snakeskin.measure._s
import org.snakeskin.registry.RealTimeTasks
import org.snakeskin.rt.RealTimeTask
import org.snakeskin.runtime.SnakeskinPlatform
import org.snakeskin.runtime.SnakeskinRuntime
import org.team401.robot2020.auto.InfiniteRechargeAuto
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.robot.TurretLimelight
import org.team401.robot2020.control.turret.TurretUpdater
import org.team401.robot2020.subsystems.*
import org.team401.taxis.diffdrive.characterization.CharacterizeDrivetrainAuto
import org.team401.taxis.diffdrive.characterization.MeasureTrackScrubFactorAuto
import org.team401.taxis.diffdrive.characterization.MeasureWheelRadiusAuto
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

@Setup
fun setup() {
    SnakeskinRuntime.createRealTimeExecutor(0.01._s)
    //Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick)
    //Subsystems.add(DrivetrainSubsystem)
    //Subsystems.add(TurretSubsystem)
    //RealTimeTasks.add(OdometryTracker(DrivetrainSubsystem))
    //RealTimeTasks.add(TurretUpdater)

    //Subsystems.add(HangingSubsystem)
    Subsystems.add(BallSubsystem)
    //TurretLimelight.start()
}