package org.team401.robot2020.config

import org.snakeskin.measure.Inches
import org.snakeskin.measure.InchesPerSecond
import org.snakeskin.measure.InchesPerSecondPerSecond

object HangerParameters {
    val sprocketPitchRadius = (1.1 / 2).Inches

    val maxVelocity = 5.0.InchesPerSecondPerSecond

    val maxAccel = 1.0.InchesPerSecond
}