package org.team401.robot2020.config.constants

import org.snakeskin.measure.*
import org.snakeskin.utility.value.SelectableDouble

object ClimbingConstants {
    //Elevator Dynamics
    val leftElevatorKs by SelectableDouble(0.417, 0.0)
    val leftElevatorKg by SelectableDouble(0.0926, 0.0)
    val leftElevatorKv by SelectableDouble(0.213, 0.0)
    val leftElevatorKa by SelectableDouble(0.0034, 0.0)

    val rightElevatorKs by SelectableDouble(0.51, 0.0)
    val rightElevatorKg by SelectableDouble(0.145, 0.0)
    val rightElevatorKv by SelectableDouble(0.222, 0.0)
    val rightElevatorKa by SelectableDouble(0.00463, 0.0)

    //Elevator Controllers
    val leftElevatorKp by SelectableDouble(6.0, 0.0)
    val rightElevatorKp by SelectableDouble(6.0, 0.0)

    //Geometric Constants
    val pitchRadius = (1.757).Inches / 2.0._ul

    val distBetweenHooks = 23.08.Inches

    val maxVelocity = 40.0.Inches / 1.75.Seconds
    val maxAngularVelocity = maxVelocity.toAngularVelocity(pitchRadius)

    val maxAccel = maxVelocity / 0.15.Seconds
    val maxAngularAccel = maxAccel.toAngularAcceleration(pitchRadius)

    val jogRate = 10.0.InchesPerSecond

    val lockPistonDelay = 0.1.Seconds

    val positionTolerance = 1.0.Inches

    const val homingPower = -.25
    val homingVelocityThreshold = 0.01.RevolutionsPerSecond
    val homingTime = 0.1.Seconds

    //Heights
    val minHeight = 1.0.Inches
    val maxHeight = 34.0.Inches
    val deployHeight = 10.0.Inches
    val climbingHeight = 1.0.Inches
}