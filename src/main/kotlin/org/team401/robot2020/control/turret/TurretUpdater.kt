package org.team401.robot2020.control.turret

import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.subsystems.ShooterSubsystem

object TurretUpdater: RealTimeTask() {
    override fun action(timestamp: TimeMeasureSeconds, dt: TimeMeasureSeconds) {
        RobotState.addTurretObservation(timestamp, ShooterSubsystem.getTurretAngle(), ShooterSubsystem.getTurretVelocity())
    }
}
