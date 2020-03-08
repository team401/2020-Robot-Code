package org.team401.robot2020.auto.steps

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2020.control.robot.RobotState

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 * Waits for the odometry (estimated, not from vision) to cross a certain threshold.  The step will unblock
 * when the field to robot pose surpasses the provided threshold, along the provided axis, in the provided direction
 */
class WaitForOdometry(val axis: Axis, val direction: Direction, val threshold: Double): AutoStep() {
    enum class Axis {
        X,
        Y,
        THETA
    }

    enum class Direction {
        POSITIVE,
        NEGATIVE
    }

    override fun entry(currentTime: TimeMeasureSeconds) {

    }

    override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds): Boolean {
        val translation = RobotState.getFieldToVehicle(currentTime).translation
        val rotation = RobotState.getFieldToVehicle(currentTime).rotation
        return when (axis) {
            Axis.X -> {
                when (direction) {
                    Direction.POSITIVE -> translation.x() >= threshold
                    Direction.NEGATIVE -> translation.x() <= threshold
                }
            }

            Axis.Y -> {
                when (direction) {
                    Direction.POSITIVE -> translation.y() >= threshold
                    Direction.NEGATIVE -> translation.y() <= threshold
                }
            }

            Axis.THETA -> {
                when (direction) {
                    Direction.POSITIVE -> rotation.degrees >= threshold
                    Direction.NEGATIVE -> rotation.degrees <= threshold
                }
            }
        }
    }

    override fun exit(currentTime: TimeMeasureSeconds) {

    }
}