package org.team401.robot2020.control.robot

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.taxis.geometry.Rotation2d

class ShooterTargetingParameters(
    val timestamp: TimeMeasureSeconds,
    val turretError: Rotation2d,
    val turretFeedVelocity: AngularVelocityMeasureRadiansPerSecond,
    val rangeToTarget: LinearDistanceMeasureInches
)