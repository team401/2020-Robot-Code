package org.team401.robot2020.config.constants

import org.snakeskin.measure.RevolutionsPerMinute
import org.snakeskin.measure.Seconds
import org.snakeskin.utility.value.SelectableDouble

object BallConstants {
    val intakeArmKs by SelectableDouble(0.758, 0.0)
    val intakeArmKv by SelectableDouble(0.431, 0.0)
    val intakeArmKcos by SelectableDouble(0.465, 0.0)
    val intakeArmKp by SelectableDouble(3.0, 0.0)

    val towerFeedingRate = 5000.0.RevolutionsPerMinute

    val towerSpacingPower = .1
    val towerReversingPower = -.5
    val towerSpacingTime = .02.Seconds

    val flyingVFeedPower = .5
    val flyingVReversePower = 0.2
}