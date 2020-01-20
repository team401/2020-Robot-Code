package org.team401.robot2020.config

import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.template.DifferentialDrivetrainGeometry

object DrivetrainGeometry : DifferentialDrivetrainGeometry {
    override val wheelRadius = 2.9615494355983927.Inches
    override val wheelbase = 22.75.Inches

}