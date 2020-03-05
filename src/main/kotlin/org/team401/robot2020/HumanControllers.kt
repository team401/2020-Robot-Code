package org.team401.robot2020

import org.snakeskin.dsl.*
import org.snakeskin.hid.channel.AxisChannel
import org.snakeskin.hid.channel.ButtonChannel
import org.snakeskin.logic.Direction
import org.team401.robot2020.control.robot.SuperstructureManager
import org.team401.robot2020.subsystems.ClimbingSubsystem
import org.team401.robot2020.subsystems.ShooterSubsystem

object HumanControllers {
    val driveTranslationChannel = AxisChannel()
    val driveRotationChannel = AxisChannel()
    val driveQuickTurnChannel = ButtonChannel()
    val turretJogChannel = AxisChannel()
    val manualShotPowerChannel = AxisChannel()

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
                SuperstructureManager.startClimb()
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

    val gamePad = HumanControls.f310(2) {
        bindAxis(Axes.RightX, turretJogChannel)
        bindAxis(Axes.LeftTrigger, manualShotPowerChannel)

        //Intake
        whenButton(Buttons.B) {
            pressed { SuperstructureManager.startIntaking() }
            released { SuperstructureManager.stopIntaking() }
        }

        //Shooter controls
        whenButton(Buttons.Y) {
            pressed { SuperstructureManager.lockFarShot() }
        }

        whenButton(Buttons.X) {
            pressed { SuperstructureManager.unwindShooter() }
        }

        whenButton(Buttons.A) {
            pressed { SuperstructureManager.lockNearShot() }
        }

        whenAxis(Axes.RightTrigger) {
            crosses(0.5) { SuperstructureManager.startFiring() }
            returns(0.5) { SuperstructureManager.stopFiring() }
        }

        //Turret manual
        whenButton(Buttons.RightStick) {
            pressed { ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.Jogging) }
            released { ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.Hold) }
        }

        //Shooter RPM memory
        whenButton(Buttons.Back) {
            pressed { ShooterSubsystem.resetFlywheelAdjust() }
        }

        whenHatChanged(Hats.DPad) {
            when (it) {
                Direction.NORTH -> ShooterSubsystem.adjustFlywheelUp()
                Direction.SOUTH -> ShooterSubsystem.adjustFlywheelDown()
            }
        }
    }
}

