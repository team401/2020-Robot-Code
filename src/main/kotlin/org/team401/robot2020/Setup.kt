package org.team401.robot2020

import org.snakeskin.dsl.*
import org.team401.robot2020.subsystems.DrivetrainSubsystem

@Setup
fun setup() {
    Controllers.add(HumanControllers.leftStick, HumanControllers.rightStick)
    Subsystems.add(DrivetrainSubsystem)
}