package org.team401.robot2020.config.constants

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.RevolutionsPerMinute
import org.snakeskin.measure.Seconds
import org.snakeskin.utility.value.SelectableDouble

object BallConstants {
    val towerKp = 0.00015
    val towerFeedingRate = 5000.0.RevolutionsPerMinute

    val towerSpacingPower = .1
    val towerReversingPower = -.5
    val towerSpacingTime = .02.Seconds
    val towerShootingPower = 0.75
    val towerManualReversePower = -.5

    val flyingVLeftIntakingPower = 1.0
    val flyingVRightIntakingPower = .75
    val flyingVShootingPower = 1.0
    val flyingVReversingPower = -1.0
    val flyingVIdlePower = 1.0

    val flyingVIdleTimeout = 3.0.Seconds
}