package org.team401.robot2020.config.constants

import com.revrobotics.ColorMatch
import edu.wpi.first.wpilibj.util.Color
import org.snakeskin.measure.*
import org.snakeskin.utility.value.SelectableDouble
import org.snakeskin.utility.value.SelectableValue
import org.team401.robot2020.config.FieldGeometry

/**
 * Various constants for the spinner subsystem.
 */
object SpinnerConstants {
    //Color values
    val blueColor: Color = ColorMatch.makeColor(0.1325, 0.458, 0.4113)
    val greenColor: Color = ColorMatch.makeColor(0.1791, 0.635, 0.185)
    val redColor: Color = ColorMatch.makeColor(0.654, 0.285, 0.0581)
    val yellowColor: Color = ColorMatch.makeColor(0.447, 0.499, 0.0537)

    //Geometric constants
    val wheelRadius by SelectableValue(1.0.Inches, 1.0.Inches) //Radius of drive wheel
    private val spinnerControlPanelRatio = (FieldGeometry.controlPanelDiameter / 2.0._ul) / wheelRadius
    private const val gearboxRatio = 10.0 / 1.0 //10:1 VP gearbox
    val effectiveRatio = spinnerControlPanelRatio * gearboxRatio //Total reduction from motor to control panel

    //Max theoretical free speed of the control panel, used as a rudimentary "Kv" constant to convert velocity to voltage
    val maxFreeSpeed = (11000.0._rev_per_min).toRadiansPerSecond() / effectiveRatio._ul

    //Profile constants (in terms of control panel velocity!)
    val cruiseVelocity = 0.75._rev_per_s.toRadiansPerSecond()
    val rotationAccel = 0.5._rev_per_s_per_s.toRadiansPerSecondPerSecond() //Acceleration for rotation objective
    val positionAccel = 0.1._rev_per_s_per_s.toRadiansPerSecondPerSecond() //Acceleration for position objective
    val rotationDistance = 4.0.Revolutions.toRadians() //Number of rotations for rotation objective

    //Time to wait before spinning to allow the piston to deploy
    val deployDelay = 0.1.Seconds
}