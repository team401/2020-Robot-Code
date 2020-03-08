package org.team401.robot2020.config.constants

import org.snakeskin.logic.scalars.IScalar
import org.snakeskin.measure.FeetPerSecond
import org.snakeskin.measure.Inches
import org.snakeskin.template.DifferentialDrivetrainGeometry
import org.snakeskin.utility.CheesyDriveController
import org.snakeskin.utility.value.SelectableDouble
import org.snakeskin.utility.value.SelectableValue
import org.team401.taxis.template.DifferentialDrivetrainDynamicsParameters

/**
 * Various constants for the drivetrain.  Some general conventions:
 * - All geometric constants are in inches
 * - All chassis dynamics constants are in SI
 * - All transmission dynamics and controller parameters are in terms of radians per second
 */
object DrivetrainConstants: DifferentialDrivetrainGeometry, DifferentialDrivetrainDynamicsParameters {
    //Geometry Constants
    override val wheelRadius by SelectableValue(2.9615494355983927.Inches, 1.0.Inches)
    override val wheelbase by SelectableValue(22.75.Inches, 1.0.Inches)

    //Chassis Dynamics Constants
    override val trackScrubFactor by SelectableDouble(1.0020927384804579, 1.0) // emp. wheelbase / geom. wheelbase
    override val angularDrag by SelectableDouble(0.0, 0.0) //unit?
    override val inertialMass by SelectableDouble(64.09, 0.0) // kg
    override val momentOfInertia by SelectableDouble(5.632, 0.0) // kg m^2

    //Transmission Dynamics Constants
    override val leftKs by SelectableDouble(0.225, 1.0) // volts
    override val leftKv by SelectableDouble(0.217, 1.0) // volts / rad/s
    override val leftKa by SelectableDouble(0.00571, 1.0) //volts / rad/s/s
    override val rightKs by SelectableDouble(0.225, 1.0)
    override val rightKv by SelectableDouble(0.211, 1.0)
    override val rightKa by SelectableDouble(0.00797, 1.0)

    //Controller Parameters
    val leftKp by SelectableDouble(.25, 0.0) // volts / (rad/s error)
    val rightKp by SelectableDouble(.25, 0.0)

    object CheesyDriveParameters: CheesyDriveController.DefaultParameters() {
        override val quickTurnScalar = object : IScalar {
            override fun scale(input: Double) = input / 2.0
        }
    }

    //Max speed of the drive, used to convert joystick values into target speeds for the drive
    val maxSpeed = 15.7.FeetPerSecond.toAngularVelocity(wheelRadius).toRadiansPerSecond()
}