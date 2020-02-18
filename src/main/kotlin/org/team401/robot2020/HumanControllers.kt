package org.team401.robot2020

import org.snakeskin.dsl.*
import org.snakeskin.hid.channel.AxisChannel
import org.snakeskin.hid.channel.ButtonChannel
import org.snakeskin.logic.Direction
import org.team401.robot2020.subsystems.BallSubsystem
import org.team401.robot2020.subsystems.ClimbingSubsystem

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

        //<editor-fold desc="Climbing Controls">
        whenButton(Buttons.StickLeft) {
            pressed {
                ClimbingSubsystem.climbingMachine.setState(ClimbingSubsystem.ClimbingStates.Deploying)
            }
        }

        whenHatChanged(Hats.StickHat) {
            when (it) {
                Direction.NORTH -> ClimbingSubsystem.climbingMachine.setState(ClimbingSubsystem.ClimbingStates.JogUp)
                Direction.SOUTH -> ClimbingSubsystem.climbingMachine.setState(ClimbingSubsystem.ClimbingStates.JogDown)
                else -> ClimbingSubsystem.climbingMachine.setState(ClimbingSubsystem.ClimbingStates.Hold)
            }
        }

        whenButton(Buttons.StickRight) {
            pressed {
                ClimbingSubsystem.climbingMachine.setState(ClimbingSubsystem.ClimbingStates.FixedClimb)
            }
        }
        //</editor-fold>
    }

    val gamePad = HumanControls.f310(0)
}

