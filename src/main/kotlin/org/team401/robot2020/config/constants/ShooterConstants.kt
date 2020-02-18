package org.team401.robot2020.config.constants

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.RevolutionsPerMinute
import org.snakeskin.measure.Seconds
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
    val turretKs by SelectableDouble(0.230764, 0.0) // volts
    val turretKv by SelectableDouble(0.799, 0.0) // volts / rad/s

    val turretKp by SelectableDouble(15.0, 15.0) // volts / (rad error)
    val turretKd by SelectableDouble(0.0, 0.0) // volts / (rad/s error)

    //Flywheel dynamics
    val flywheelRatio = 18.0 / 30.0 // Total reduction of motor to flywheel shaft

    val flywheelKs by SelectableDouble(0.134, 0.0) // volts
    val flywheelKv by SelectableDouble(0.0738467, 0.0) // volts / rev/s

    val flywheelKp by SelectableDouble(0.0015, 0.0) // SPARK MAX p unit
    val flywheelKd by SelectableDouble(0.0, 0.0) // SPARK MAX d unit

    //Profiled acceleration ramp of the flywheel
    val flywheelRampAcceleration = (8000.0.RevolutionsPerMinute / 2.0.Seconds).toRevolutionsPerSecondPerSecond()
}