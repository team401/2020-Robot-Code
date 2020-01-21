package org.team401.robot2020

import org.snakeskin.dsl.*
import org.snakeskin.hid.channel.AxisChannel
import org.snakeskin.hid.channel.ButtonChannel
import org.snakeskin.logic.scalars.LowPassScalar
import org.team401.robot2020.subsystems.HangingSubsystem
import org.team401.robot2020.subsystems.SpinnerSubsystem

object HumanControllers {
    val driveTranslationChannel = AxisChannel()
    val driveRotationChannel = AxisChannel()
    val driveQuickTurnChannel = ButtonChannel()
    val driveShiftChannel = ButtonChannel()

    val leftStick = HumanControls.t16000m(0) {
        invertAxis(Axes.Pitch)
        bindAxis(Axes.Pitch, driveTranslationChannel)
        bindButton(Buttons.Trigger, driveShiftChannel)
    }

    val rightStick = HumanControls.t16000m(1) {
        bindAxis(Axes.Roll, driveRotationChannel)
        bindButton(Buttons.Trigger, driveQuickTurnChannel)
    }

    val gamepad = HumanControls.f310(2) {
        whenButton(Buttons.Y) {
            pressed {
                HangingSubsystem.HangingMachine.setState(HangingSubsystem.States.Forward)
            }

            released {
                HangingSubsystem.HangingMachine.setState(HangingSubsystem.States.Disabled)
            }
        }

        whenButton(Buttons.A) {
            pressed {
                HangingSubsystem.HangingMachine.setState(HangingSubsystem.States.Reversed)
            }

            released {
                HangingSubsystem.HangingMachine.setState(HangingSubsystem.States.Disabled)
            }
        }
    }
}
