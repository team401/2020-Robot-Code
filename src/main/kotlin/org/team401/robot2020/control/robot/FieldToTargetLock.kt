package org.team401.robot2020.control.robot

import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.taxis.geometry.Pose2d

/**
 * Represents a specific capture of the field to target transform with a given timestamp
 */
data class FieldToTargetLock(
    val timestamp: TimeMeasureSeconds,
    val fieldToTarget: Pose2d
)