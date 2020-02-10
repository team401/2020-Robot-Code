package org.team401.robot2020.config

import org.snakeskin.measure.RevolutionsPerMinute
import org.snakeskin.measure.Seconds

object FlywheelDynamics {
    val gearRatio = 18.0 / 30.0

    val Ks = 0.134 // volts
    val Kv = 0.0738467 // volts / rev/s

    val Kp = .001

    val maxAcceleration = (8000.0.RevolutionsPerMinute / 2.0.Seconds).toRevolutionsPerSecondPerSecond()
}