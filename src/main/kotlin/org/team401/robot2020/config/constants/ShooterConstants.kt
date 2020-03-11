package org.team401.robot2020.config.constants

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.utility.value.SelectableDouble
import org.snakeskin.utility.value.SelectableValue
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.util.PolynomialRegression
import org.team401.util.TrapezoidProfileLite

/**
 * Various constants for the shooter.  Some general conventions:
 * - All geometric constants are in inches or degrees
 * - All turret dynamics and controller constants are in terms of radians (per second)
 * - All flywheel dynamics and controller constants are in terms of revolutions per second
 */
object ShooterConstants {
    //Turret geometry
    val robotToTurret by SelectableValue( //Translation from robot origin to turret origin
        Translation2d(-9.5, 0.0),
        Translation2d(-9.5, 0.0)
    )

    val turretCameraNearOriginToLens by SelectableValue(
        Pose2d(-1.5, 0.0, Rotation2d.fromDegrees(0.0)),
        Pose2d()
    )

    val turretCameraFarOriginToLens by SelectableValue(
        Pose2d(1.25, 0.0, Rotation2d.fromDegrees(0.0)),
        Pose2d()
    )

    val turretCameraNearLensHeight by SelectableValue(
        41.5.Inches,
        0.0.Inches
    )

    val turretCameraFarLensHeight by SelectableValue(
        42.5.Inches,
        0.0.Inches
    )

    val turretCameraNearHorizontalPlaneToLens by SelectableValue(
        Rotation2d.fromDegrees(29.7),
        Rotation2d.fromDegrees(0.0)
    )

    val turretCameraFarHorizontalPlaneToLens by SelectableValue(
        Rotation2d.fromDegrees(9.12),
        Rotation2d.fromDegrees(0.0)
    )

    //Turret dynamics
    val turretKs by SelectableDouble(0.236, 0.0) // volts
    val turretKv by SelectableDouble(1.07, 0.0) // volts / rad/s
    val turretKa by SelectableDouble(0.00497, 0.0) // volts / rad/s/s

    val turretKp by SelectableDouble(30.0, 0.0) // volts / (rad error)
    val turretKd by SelectableDouble(1.0, 0.0) // volts / (rad/s error)
    val turretTrackingKp by SelectableDouble(17.0, 0.0)
    val turretTrackingKd by SelectableDouble(0.1, 0.0)

    val turretVelocity = (180.0.Degrees / 0.3.Seconds).toRadiansPerSecond()
    val turretAccel = turretVelocity / 0.05.Seconds

    val turretNegativeLimit = (-20.0).Degrees.toRadians()
    val turretPositiveLimit = 1.0.Revolutions.toRadians() + 20.0.Degrees.toRadians()
    val turretRapidError = 20.0.Degrees.toRadians()

    val turretJogRate = 0.25.RevolutionsPerSecond.toRadiansPerSecond()

    val turretTrackingLookahead = 0.7.Seconds

    val turretRatio = 190.0 / 18.0 // Total reduction from gearbox shaft to turret axis

    //Flywheel dynamics
    val flywheelRatio = 18.0 / 30.0 // Total reduction of motor to flywheel shaft

    val flywheelKs by SelectableDouble(0.324, 0.0) // volts
    val flywheelKv by SelectableDouble(0.0106, 0.0) // volts / rev/s
    val flywheelKa by SelectableDouble(0.000438, 0.0) //volts / rad/s/s

    val flywheelKp by SelectableDouble(0.3, 0.0) // Talon FX p unit
    val flywheelKi by SelectableDouble(0.0, 0.0)
    val flywheelKd by SelectableDouble(5.0, 0.0)

    //Profiled acceleration ramp of the flywheel
    val flywheelRampAcceleration = (8000.0.RevolutionsPerMinute / 2.0.Seconds).toRadiansPerSecondPerSecond()

    val flywheelNearConstantCorrection = 0.0.RevolutionsPerMinute.toRadiansPerSecond()
    val flywheelFarConstantCorrection = 0.0.RevolutionsPerMinute.toRadiansPerSecond()
    val flywheelFarShotSpeed = (9750.0.RevolutionsPerMinute).toRadiansPerSecond()
    val flywheelMinimumSpeed = (1000.0.RevolutionsPerMinute).toRadiansPerSecond() //Min speed to allow manual adjustment to reach

    //Rate to bump the flywheel speed up or down by
    val flywheelBump = 50.0.RevolutionsPerMinute.toRadiansPerSecond()

    //LUT and regression
    val flywheelLUT = arrayOf(
        doubleArrayOf(101.0, 3600.0),
        doubleArrayOf(142.0, 3785.0),
        doubleArrayOf(183.0, 4050.0),
        doubleArrayOf(220.0, 4360.0),
        doubleArrayOf(254.0, 4450.0)
    )

    val flywheelRegression = PolynomialRegression(flywheelLUT, 2)
}