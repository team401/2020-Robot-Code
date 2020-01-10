package org.team401.robot2020.config

import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.template.DifferentialDrivetrainGeometry

object Geometry {
    object DrivetrainGeometry : DifferentialDrivetrainGeometry {
        override val wheelRadius = 0.0.Inches
        override val wheelbase = 0.0.Inches

    }
}