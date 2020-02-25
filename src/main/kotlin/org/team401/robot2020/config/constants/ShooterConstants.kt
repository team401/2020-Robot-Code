package org.team401.robot2020.config.constants

import org.snakeskin.measure.*
import org.snakeskin.utility.value.SelectableDouble
import org.snakeskin.utility.value.SelectableValue
import org.team401.taxis.geometry.Translation2d

/**
 * Various constants for the shooter.  Some general conventions:
 * - All geometric constants are in inches or degrees
 * - All turret dynamics and controller constants are in terms of radians (per second)
 * - All flywheel dynamics and controller constants are in terms of revolutions per second
 */
object ShooterConstants {
    //Turret geometry
    val robotToTurret by SelectableValue( //Translation from robot origin to turret origin
        Translation2d(10.0, 0.0),
        Translation2d(10.0, 0.0)
    )

    val turretCameraMountingAngleClose by SelectableValue(15.0.Degrees, 15.0.Degrees) //Angle of the turret camera in close mode
    val turretCameraMountingAngleFar by SelectableValue(15.0.Degrees, 15.0.Degrees) //Angle of the turret camera in far mode
    val turretCameraMountingHeightClose by SelectableValue(5.0.Inches, 5.0.Inches) //Height of the turret camera from the floor in close mode
    val turretCameraMountingHeightFar by SelectableValue(5.0.Inches, 5.0.Inches) //Height of the turret camera from the floor in far mode

    //Turret dynamics
    val turretKs by SelectableDouble(0.236, 0.0) // volts
    val turretKv by SelectableDouble(1.07, 0.0) // volts / rad/s
    val turretKa by SelectableDouble(0.00497, 0.0) // volts / rad/s/s

    val turretKp by SelectableDouble(30.0, 0.0) // volts / (rad error)
    val turretKd by SelectableDouble(1.0, 0.0) // volts / (rad/s error)

    val turretVelocity = (180.0.Degrees / 0.3.Seconds).toRadiansPerSecond()
    val turretAccel = turretVelocity / 0.05.Seconds

    val turretRatio = 190.0 / 18.0 // Total reduction from gearbox shaft to turret axis

    //Flywheel dynamics
    val flywheelRatio = 18.0 / 30.0 // Total reduction of motor to flywheel shaft

    val flywheelKs by SelectableDouble(0.216, 0.0) // volts
    val flywheelKv by SelectableDouble(0.0128, 0.0) // volts / rev/s
    val flywheelKa by SelectableDouble(0.00429, 0.0) //volts / rad/s/s

    val flywheelKp by SelectableDouble(0.07, 0.0) // SPARK MAX p unit

    val flywheelMaxVelocity = 9000.0.RevolutionsPerMinute.toRadiansPerSecond()

    //Profiled acceleration ramp of the flywheel
    val flywheelRampAcceleration = (8000.0.RevolutionsPerMinute / 2.0.Seconds).toRadiansPerSecondPerSecond()

    //Default speed for the flywheel to spin at
    val flywheelDefaultSpeed = (6000.0.RevolutionsPerMinute).toRadiansPerSecond()

    //Rate to bump the flywheel speed up or down by
    val flywheelBump = 10.0.RevolutionsPerSecond.toRadiansPerSecond()

    //LUT and regression
    val flywheelLUT = arrayOf(
        doubleArrayOf(0.0, 0.0),
        doubleArrayOf(1.0, 10.0)
    )
}