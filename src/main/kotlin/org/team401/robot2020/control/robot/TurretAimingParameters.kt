package org.team401.robot2020.control.robot

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.taxis.geometry.Rotation2d

data class TurretAimingParameters(val desiredAngle: Rotation2d, val distanceToGoal: LinearDistanceMeasureInches)