package org.team401.robot2020.config.constants

import org.snakeskin.measure.Inches
import org.snakeskin.measure.InchesPerSecond
import org.snakeskin.measure.InchesPerSecondPerSecond

object ClimbingConstants {
    const val kP = 0.0
    const val kS = 0.0
    const val kV = 0.0

    val sprocketPitchDiameter = (1.757).Inches

    val distBetweeHooks = 23.08.Inches

    val maxVelocity = 5.0.InchesPerSecond

    val maxAccel = 1.0.InchesPerSecondPerSecond
}