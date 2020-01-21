package org.team401.robot2020

import org.snakeskin.auto.AutoManager
import org.snakeskin.dsl.*
import org.snakeskin.measure.Inches
import org.snakeskin.measure._s
import org.snakeskin.registry.RealTimeTasks
import org.snakeskin.rt.RealTimeTask
import org.snakeskin.runtime.SnakeskinRuntime
import org.team401.robot2020.auto.InfiniteRechargeAuto
import org.team401.robot2020.control.turret.TurretUpdater
import org.team401.robot2020.subsystems.*
import org.team401.taxis.diffdrive.characterization.CharacterizeDrivetrainAuto
import org.team401.taxis.diffdrive.characterization.MeasureTrackScrubFactorAuto
import org.team401.taxis.diffdrive.characterization.MeasureWheelRadiusAuto
import org.team401.taxis.diffdrive.odometry.OdometryTracker

@Setup
fun setup() {
    SnakeskinRuntime.createRealTimeExecutor(0.01._s)
    Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick)
    Subsystems.add(DrivetrainSubsystem)
    Subsystems.add(TurretSubsystem)
    RealTimeTasks.add(OdometryTracker(DrivetrainSubsystem), TurretUpdater)
}