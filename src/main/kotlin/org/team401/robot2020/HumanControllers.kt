package org.team401.robot2020

import org.snakeskin.dsl.*
import org.snakeskin.hid.channel.AxisChannel
import org.snakeskin.hid.channel.ButtonChannel
import org.snakeskin.logic.scalars.LowPassScalar
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
        whenButton(Buttons.A) {
            pressed {
                SpinnerSubsystem.SpinnerMachine.setState(SpinnerSubsystem.States.Position)
            }

            released {
                SpinnerSubsystem.SpinnerMachine.setState(SpinnerSubsystem.States.Disabled)
            }

            whenButton(Buttons.B) {
                pressed {
                    SpinnerSubsystem.SpinnerMachine.setState(SpinnerSubsystem.States.Rotation)
                }

                released {
                    SpinnerSubsystem.SpinnerMachine.setState(SpinnerSubsystem.States.Disabled)
                }
            }
        }
    }
}
