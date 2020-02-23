package org.team401.robot2020.util

import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.team401.taxis.util.Util

fun ProfiledPIDController.getAcceleration(constraints: TrapezoidProfile.Constraints, lastSetpoint: TrapezoidProfile.State): Double {
    if (Util.epsilonEquals(constraints.maxVelocity, setpoint.velocity)) return 0.0 //Zero accel case
    if (setpoint.velocity > lastSetpoint.velocity) return constraints.maxAcceleration //Positive acceleration case
    if (setpoint.velocity < lastSetpoint.velocity) return -1.0 * constraints.maxAcceleration //Negative acceleration case
    return 0.0 //Unknown case, return 0 for safety
}