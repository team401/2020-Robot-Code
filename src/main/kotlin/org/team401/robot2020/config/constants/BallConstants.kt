package org.team401.robot2020.config.constants

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.RevolutionsPerMinute
import org.snakeskin.measure.Seconds
import org.snakeskin.utility.value.SelectableDouble

object BallConstants {
    val intakeArmKs by SelectableDouble(0.758, 0.0)
    val intakeArmKv by SelectableDouble(0.431, 0.0)
    val intakeArmKcos by SelectableDouble(0.465, 0.0)
    val intakeArmKp by SelectableDouble(6.0, 0.0)
    val intakeArmVelocity =  (180.0.Degrees / .75.Seconds).toRadiansPerSecond()
    val intakeArmAcceleration = intakeArmVelocity / 0.2.Seconds

    val towerKp = 0.00015
    val towerFeedingRate = 5000.0.RevolutionsPerMinute

    val towerSpacingPower = .1
    val towerReversingPower = -.5
    val towerSpacingTime = .02.Seconds
    val towerShootingPower = 0.75
    val towerManualReversePower = -.5


    val flyingVIntakingPower = 1.0
    val flyingVShootingPower = 1.0
    val flyingVReversingPower = -1.0
    val flyingVIdlePower = 1.0

    val flyingVIdleTimeout = 3.0.Seconds
}