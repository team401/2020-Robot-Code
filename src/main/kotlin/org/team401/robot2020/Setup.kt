package org.team401.robot2020

import org.snakeskin.dsl.*
import org.snakeskin.measure._s
import org.snakeskin.runtime.SnakeskinRuntime
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.robot2020.subsystems.FlywheelSubsystem

@Setup
fun setup() {
    SnakeskinRuntime.createRealTimeExecutor(0.01._s)
    //Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick)
    //Subsystems.add(DrivetrainSubsystem)
    Subsystems.add(FlywheelSubsystem)
}