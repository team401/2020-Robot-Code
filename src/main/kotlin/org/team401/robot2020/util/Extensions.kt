package org.team401.robot2020.util

import org.team401.taxis.util.Util
import org.team401.util.ProfiledPIDControllerLite
import org.team401.util.TrapezoidProfileLite

fun ProfiledPIDControllerLite.getAcceleration(lastSetpoint: TrapezoidProfileLite.State, constraints: TrapezoidProfileLite.Constraints = TrapezoidProfileLite.Constraints(0.0, 0.0)): Double {
    if (Util.epsilonEquals(this.constraints.maxVelocity, setpoint.velocity)) return 0.0 //Zero accel case
    if (setpoint.velocity > lastSetpoint.velocity) return this.constraints.maxAcceleration //Positive acceleration case
    if (setpoint.velocity < lastSetpoint.velocity) return -1.0 * this.constraints.maxAcceleration //Negative acceleration case
    return 0.0 //Unknown case, return 0 for safety
}