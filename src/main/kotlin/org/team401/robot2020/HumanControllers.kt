package org.team401.robot2020

import org.snakeskin.dsl.*
import org.snakeskin.hid.channel.AxisChannel
import org.snakeskin.hid.channel.ButtonChannel
import org.snakeskin.logic.scalars.LowPassScalar
import org.team401.robot2020.subsystems.BallSubsystem
import org.team401.robot2020.subsystems.SpinnerSubsystem

object HumanControllers {
    val driveTranslationChannel = AxisChannel()
    val driveRotationChannel = AxisChannel()
    val driveQuickTurnChannel = ButtonChannel()

    val leftStick = HumanControls.t16000m(0) {
        invertAxis(Axes.Pitch)
        bindAxis(Axes.Pitch, driveTranslationChannel)
    }

    val rightStick = HumanControls.t16000m(1) {
        bindAxis(Axes.Roll, driveRotationChannel)
        bindButton(Buttons.Trigger, driveQuickTurnChannel)
    }

    val gamePad = HumanControls.f310(0) {
        whenButton(Buttons.A) {
            pressed {
                println("A pressed")
                BallSubsystem.intakingMachine.setState(BallSubsystem.IntakeStates.Intaking)
            }

            released {
                println("A released")
                BallSubsystem.intakingMachine.setState(BallSubsystem.IntakeStates.Waiting)
            }
        }
    }
}

