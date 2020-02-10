package org.team401.robot2020.control

import org.snakeskin.measure.RevolutionsPerSecond
import org.snakeskin.measure.acceleration.angular.AngularAccelerationMeasureRevolutionsPerSecondPerSecond
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRevolutionsPerSecond

class VelocityProfiler(val maxAcceleration: AngularAccelerationMeasureRevolutionsPerSecondPerSecond) {
    private var currentCmd = 0.0.RevolutionsPerSecond

    fun reset() {
        currentCmd = 0.0.RevolutionsPerSecond
    }

    fun calculate(dt: TimeMeasureSeconds, velocityCommand: AngularVelocityMeasureRevolutionsPerSecond): AngularVelocityMeasureRevolutionsPerSecond {
        val dv = dt * maxAcceleration

        if (velocityCommand > currentCmd) {
            currentCmd += dv
            if (currentCmd > velocityCommand) currentCmd = velocityCommand
        } else if (velocityCommand < currentCmd) {
            currentCmd -= dv
            if (currentCmd < velocityCommand) currentCmd = velocityCommand
        }

        return currentCmd
    }
}