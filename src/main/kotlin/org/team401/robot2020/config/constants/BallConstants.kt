package org.team401.robot2020.config.constants

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.RevolutionsPerMinute
import org.snakeskin.measure.Seconds
import org.snakeskin.utility.value.SelectableDouble

object BallConstants {
    val towerFeedingRate = 5000.0.RevolutionsPerMinute

    val towerSpacingPower = .25
    val towerReversingPower = -.5
    val towerFeedingPower = 0.4
    val towerSpacingTime = .02.Seconds
    val towerShootingPower = .8
    val towerManualReversePower = -1.0

    val flyingVLeftIntakingPower = 1.0
    val flyingVRightIntakingPower = .75

    val flyingVIdleTimeout = 3.0.Seconds
}