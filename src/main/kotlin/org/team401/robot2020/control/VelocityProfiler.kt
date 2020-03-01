package org.team401.robot2020.control

import org.snakeskin.measure.*
import org.snakeskin.measure.acceleration.angular.AngularAccelerationMeasureRadiansPerSecondPerSecond
import org.snakeskin.measure.acceleration.angular.AngularAccelerationMeasureRevolutionsPerSecondPerSecond
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRevolutionsPerSecond
import org.snakeskin.utility.value.AsyncBoolean

/**
 * Implements a simple linear velocity ramp for controlling simple systems such as a flywheel.
 */
class VelocityProfiler(private val acceleration: AngularAccelerationMeasureRadiansPerSecondPerSecond) {
    /**
     * The profiled velocity command
     */
    var velocityCommand = 0.0.RadiansPerSecond
        private set

    /**
     * The profiled acceleration command
     */
    var accelerationCommand = 0.0.RadiansPerSecondPerSecond
        private set

    /**
     * Resets the velocity command to the given value, and zeroes the acceleration
     */
    fun reset(currentSpeed: AngularVelocityMeasureRadiansPerSecond = 0.0.RadiansPerSecond) {
        velocityCommand = currentSpeed
        accelerationCommand = 0.0.RadiansPerSecondPerSecond
    }

    /**
     * Updates the values of velocityCommand and accelerationCommand based on the given inputs
     */
    fun calculate(dt: TimeMeasureSeconds, goalVelocity: AngularVelocityMeasureRadiansPerSecond) {
        val dv = dt * acceleration

        if (goalVelocity > velocityCommand) { //Goal is above current command, accelerate positively
            velocityCommand += dv
            accelerationCommand = acceleration
            if (velocityCommand > goalVelocity) {
                velocityCommand = goalVelocity
                accelerationCommand = 0.0.RadiansPerSecondPerSecond
            }
        } else if (goalVelocity < velocityCommand) { //Goal is below current command, accelerate negatively
            velocityCommand -= dv
            accelerationCommand = acceleration * (-1.0)._ul
            if (velocityCommand < goalVelocity) {
                velocityCommand = goalVelocity
                accelerationCommand = 0.0.RadiansPerSecondPerSecond
            }
        } else { //Goal is equal to current command, no acceleration
            accelerationCommand = 0.0.RadiansPerSecondPerSecond
        }
    }
}