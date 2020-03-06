package org.team401.robot2020.config.constants

import org.snakeskin.measure.Seconds
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Translation2d

object RobotConstants {
    const val COMP_INDEX = 0
    const val PRACTICE_INDEX = 1

    val rtPeriod = 0.01.Seconds

    val bumperRightToOrigin = Pose2d.fromTranslation(Translation2d(0.0, 17.0)) //Bumpers are 34 inches across
    val intakeToRobotOrigin = Pose2d.fromTranslation(Translation2d(0.0, 0.0))
}