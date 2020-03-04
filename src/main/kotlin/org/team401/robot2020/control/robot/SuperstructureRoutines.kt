package org.team401.robot2020.control.robot

import org.team401.robot2020.subsystems.BallSubsystem
import org.team401.robot2020.subsystems.ClimbingSubsystem
import org.team401.robot2020.subsystems.ShooterSubsystem

/**
 * Manages "superstructure" tasks for the robot.  This includes managing the state of the intake, hopper, tower,
 * and shooter.  Effectively acts as supervisory logic to ensure that all of the systems mentioned are in the
 * correct state
 */
object SuperstructureRoutines {
    @Synchronized fun unwindShooter() {
        ShooterSubsystem.flywheelMachine.disable()
        ShooterSubsystem.kickerMachine.disable()
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.LockToZero)
        ShooterSubsystem.setHoodState(false)
    }

    @Synchronized fun lockNearShot() {
        ShooterSubsystem.flywheelMachine.setState(ShooterSubsystem.FlywheelStates.NearShotSpinUp)
        ShooterSubsystem.kickerMachine.setState(ShooterSubsystem.KickerStates.Kick)
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.FieldRelativeTarget)
        ShooterSubsystem.setHoodState(false)
        VisionManager.turretVisionNearTargeting()
    }

    @Synchronized fun lockFarShot() {
        ShooterSubsystem.flywheelMachine.setState(ShooterSubsystem.FlywheelStates.FarShotSpinUp)
        ShooterSubsystem.kickerMachine.setState(ShooterSubsystem.KickerStates.Kick)
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.FieldRelativeTarget)
        ShooterSubsystem.setHoodState(true)
        VisionManager.turretVisionFarTargeting()
    }

    @Synchronized fun startFiring() {

    }


    @Synchronized fun startIntaking() {
        BallSubsystem.intakeMachine.setState(BallSubsystem.IntakeStates.Intake)
        BallSubsystem.flyingVMachine.setState(BallSubsystem.FlyingVStates.Intaking)
    }

    @Synchronized fun stopIntaking() {
        BallSubsystem.intakeMachine.setState(BallSubsystem.IntakeStates.GoToStow)
        BallSubsystem.flyingVMachine.setState(BallSubsystem.FlyingVStates.Idle)
    }

    @Synchronized fun startClimb() {
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.LockForClimb)
        ShooterSubsystem.flywheelMachine.disable()
        ClimbingSubsystem.climbingMachine.setState(ClimbingSubsystem.ClimbingStates.Deploying)
        BallSubsystem.intakeMachine.setState(BallSubsystem.IntakeStates.Stowed)
        BallSubsystem.flyingVMachine.disable()
        BallSubsystem.towerMachine.disable()
    }

    @Synchronized fun prepareForShooting() {
        VisionManager.turretVisionNearTargeting()
        //ShooterSubsystem.flywheelMachine.setState(ShooterSubsystem.FlywheelStates.PreSpin)
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.FieldRelativeTarget)
        ShooterSubsystem.kickerMachine.setState(ShooterSubsystem.KickerStates.Kick)
    }

    @Synchronized fun fireShooter() {
        ShooterSubsystem.kickerMachine.setState(ShooterSubsystem.KickerStates.Kick)
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.Hold)
        BallSubsystem.towerMachine.setState(BallSubsystem.TowerStates.Shooting)
        BallSubsystem.flyingVMachine.setState(BallSubsystem.FlyingVStates.Shooting)
    }

    @Synchronized fun stopShooting() {
        VisionManager.turretVisionOff()
        ShooterSubsystem.flywheelMachine.disable()
        ShooterSubsystem.kickerMachine.disable()
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.LockToZero)
        BallSubsystem.towerMachine.setState(BallSubsystem.TowerStates.Waiting)
        BallSubsystem.flyingVMachine.disable()
    }
}