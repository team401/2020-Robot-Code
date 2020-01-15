package org.team401.robot2020.config

import com.revrobotics.ColorMatch
import edu.wpi.first.wpilibj.util.Color
import org.snakeskin.measure.*

object SpinnerParameters {
    val blueTarget: Color = ColorMatch.makeColor(0.0, 0.0, 255.0)
    val greenTarget: Color = ColorMatch.makeColor(0.0, 255.0, 0.0)
    val redTarget: Color = ColorMatch.makeColor(255.0, 0.0, 0.0)
    val yellowTarget: Color = ColorMatch.makeColor(255.0, 255.0, 0.0)

    val controlPanelRadius = 16.0._in
    val spinnerWheelRadius = 1.0._in

    val spinnerToControlPanelReduction = (controlPanelRadius / spinnerWheelRadius).value
    val motorToSpinnerReduction = 10.0 //10:1 reduction gearbox
    val finalReduction = spinnerToControlPanelReduction * motorToSpinnerReduction

    val maxRps = (11000.0._rev_per_min.toRevolutionsPerSecond()).value / finalReduction //11000 is max RPM of a NEO 550

    val desiredControlPanelVelocity = 0.75._rev_per_s
    val desiredControlPanelAccel = 0.2._rev_per_s_per_s
    val desiredNumControlPanelRevs = 4.0._rev

    val decelNumRevs = (desiredNumControlPanelRevs.value - (desiredControlPanelVelocity.value * desiredControlPanelVelocity.value) / (2.0 * desiredControlPanelAccel.value))._rev
}