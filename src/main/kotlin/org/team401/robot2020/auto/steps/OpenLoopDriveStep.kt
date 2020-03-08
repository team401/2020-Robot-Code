package org.team401.robot2020.auto.steps

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2020.subsystems.DrivetrainSubsystem

class OpenLoopDriveStep(val power: Double, val time: TimeMeasureSeconds): AutoStep() {
    private var startTime = 0.0.Seconds

    override fun entry(currentTime: TimeMeasureSeconds) {
        startTime = currentTime
    }

    override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds): Boolean {
        if (currentTime - startTime >= time) return true
        DrivetrainSubsystem.tank(power, power)
        return false
    }

    override fun exit(currentTime: TimeMeasureSeconds) {
        DrivetrainSubsystem.stop()
    }
}