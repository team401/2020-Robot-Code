package org.team401.robot2020.control.robot

import org.team401.robot2020.subsystems.BallSubsystem
import org.team401.robot2020.subsystems.ClimbingSubsystem
import org.team401.robot2020.subsystems.ShooterSubsystem

/**
 * Manages "superstructure" tasks for the robot.  This includes managing the state of the intake, hopper, tower,
 * and shooter.  Effectively acts as supervisory logic to ensure that all of the systems mentioned are in the
 * correct state
 */
object SuperstructureManager {
    private var isLocked = false
    private var isShooting = false

    @Synchronized fun reset() {
        isLocked = false
        isShooting = false
    }

    @Synchronized fun unwindShooter() {
        isLocked = false
        isShooting = false
        ShooterSubsystem.flywheelMachine.disable()
        ShooterSubsystem.kickerMachine.disable()
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.LockToZero)
        ShooterSubsystem.setHoodState(false)
        VisionManager.turretVisionOff()
    }

    @Synchronized fun lockNearShot() {
        isLocked = true
        isShooting = false
        ShooterSubsystem.flywheelMachine.setState(ShooterSubsystem.FlywheelStates.NearShotSpinUp)
        ShooterSubsystem.kickerMachine.setState(ShooterSubsystem.KickerStates.Kick)
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.FieldRelativeTarget)
        ShooterSubsystem.setHoodState(false)
        VisionManager.turretVisionNearTargeting()
    }

    @Synchronized fun lockFarShot() {
        isLocked = true
        isShooting = false
        ShooterSubsystem.flywheelMachine.setState(ShooterSubsystem.FlywheelStates.FarShotSpinUp)
        ShooterSubsystem.kickerMachine.setState(ShooterSubsystem.KickerStates.Kick)
        ShooterSubsystem.turretMachine.setState(ShooterSubsystem.TurretStates.FieldRelativeTarget)
        ShooterSubsystem.setHoodState(true)
        VisionManager.turretVisionFarTargeting()
    }

    @Synchronized fun releaseVisionLock() {
        if (isLocked) {
            VisionManager.turretVisionOff()
        }
    }

    @Synchronized fun startFiring() {
        if (isLocked) {
            BallSubsystem.towerMachine.setState(BallSubsystem.TowerStates.Shooting)
            BallSubsystem.flyingVMachine.setState(BallSubsystem.FlyingVStates.Shooting)
            isShooting = true
        }
    }

    @Synchronized fun stopFiring() {
        if (isShooting) {
            BallSubsystem.towerMachine.setState(BallSubsystem.TowerStates.Waiting)
            BallSubsystem.flyingVMachine.setState(BallSubsystem.FlyingVStates.Idle)
            isShooting = false
        }
    }

    @Synchronized fun startIntaking() {
        BallSubsystem.intakeMachine.setState(BallSubsystem.IntakeStates.Intake)
        BallSubsystem.flyingVMachine.setState(BallSubsystem.FlyingVStates.Intaking)
    }

    @Synchronized fun stopIntaking() {
        BallSubsystem.intakeMachine.setState(BallSubsystem.IntakeStates.Stowed)
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
}