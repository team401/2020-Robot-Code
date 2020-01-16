package org.team401.robot2020.config

import com.revrobotics.ColorMatch
import edu.wpi.first.wpilibj.util.Color
import org.snakeskin.measure.*

object SpinnerParameters {
    val blueTarget: Color = ColorMatch.makeColor(0.1325, 0.458, 0.4113)
    val greenTarget: Color = ColorMatch.makeColor(0.1791, 0.635, 0.185)
    val redTarget: Color = ColorMatch.makeColor(0.654, 0.285, 0.0581)
    val yellowTarget: Color = ColorMatch.makeColor(0.447, 0.499, 0.0537)

    val controlPanelRadius = 16.0._in
    val spinnerWheelRadius = 1.0._in

    val spinnerToControlPanelReduction = (controlPanelRadius.value / spinnerWheelRadius.value)
    val motorToSpinnerReduction = 10.0 //10:1 reduction gearbox
    val finalReduction = spinnerToControlPanelReduction * motorToSpinnerReduction

    val maxRps = (11000.0._rev_per_min.toRevolutionsPerSecond()).value / finalReduction //11000 is max RPM of a NEO 550

    val desiredControlPanelVelocity = 0.75._rev_per_s
    val desiredControlPanelRotationAccel = 0.5._rev_per_s_per_s
    val desiredControlPanelPositionAccel = 0.1._rev_per_s_per_s
    val desiredNumControlPanelRevs = 4.0._rev
}