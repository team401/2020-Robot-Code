package org.team401.robot2020.control.robot

import org.snakeskin.measure.*
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.subsystems.ShooterSubsystem
import org.team401.sevision.VisionConstants

object SuperstructureManager : RealTimeTask() {
    enum class TurretAngleMode {
        RobotRelative,
        FieldRelative,
        Vision,
        Jog,
        Hold
    }

    var activeControlMode = TurretAngleMode.Hold
        @Synchronized get
        @Synchronized set

    var turretPositionSetpoint = ShooterConstants.turretZeroOffset
    var turretVelocitySetpoint = 0.0.RadiansPerSecond
    var distanceFromTarget = 0.0.Inches

    @Synchronized override fun action(timestamp: TimeMeasureSeconds, dt: TimeMeasureSeconds) {
        //Report turret angle to RobotState
        RobotState.addTurretObservation(timestamp, ShooterSubsystem.getTurretAngle())
        updateTurretFromVision(timestamp)
    }

    private fun updateTurretFromVision(timestamp: TimeMeasureSeconds) {
        val latestAimingParameters = RobotState.getAimingParameters(
            timestamp,
            -1, VisionConstants.kMaxGoalTrackAge
        ) ?: return

        val lookaheadTime = 0.7.Seconds

        val vehicleToTurretNow = RobotState.getVehicleToTurret(timestamp)

        val fieldToTurret = RobotState.getFieldToTurret(timestamp)
        val fieldToPredictedTurret = RobotState.getPredictedFieldToVehicle(lookaheadTime)
            .transformBy(vehicleToTurretNow) // field -> predicted turret

        val turretToPredictedTurret = fieldToTurret
            .inverse() // turret -> field
            .transformBy(fieldToPredictedTurret) // turret -> predicted turret

        val predictedTurretToGoal = turretToPredictedTurret
            .inverse() // Predicted turret -> turret
            .transformBy(latestAimingParameters.turretToGoal) // Predicted turret -> goal

        val correctedRangeToTarget = predictedTurretToGoal.translation.norm()

        val turretError = vehicleToTurretNow.rotation
            .inverse()
            .rotateBy(latestAimingParameters.robotToGoalRotation)

        distanceFromTarget = correctedRangeToTarget.Inches
        turretPositionSetpoint = ShooterSubsystem.getTurretAngle() + turretError.radians.Radians

        val velocity = RobotState.vehicleVelocityMeasured
        turretVelocitySetpoint =
            (-((latestAimingParameters.robotToGoalRotation.sin() * velocity.dx / latestAimingParameters.range) + velocity.dtheta)).RadiansPerSecond

        if (turretPositionSetpoint > (420.0.Degrees.toRadians())) {
            turretPositionSetpoint = 420.0.Degrees.toRadians()
            turretVelocitySetpoint = 0.0.RadiansPerSecond
        }

        if (turretPositionSetpoint < (0.0.Radians)) {
            turretPositionSetpoint = 0.0.Radians
            turretVelocitySetpoint = 0.0.RadiansPerSecond
        }
    }
}