package org.team401.robot2020.config

import org.snakeskin.measure.Feet
import org.snakeskin.measure.Inches
import org.team401.taxis.geometry.Translation2d

/**
 * Geometry constants of the field
 */
object FieldGeometry {
    val fieldLines = listOf(
        Translation2d(24.894, 0.0) to Translation2d(0.0, 68.533), //Right DS Line
        Translation2d(0.0, 250.533) to Translation2d(24.894, 319.067), //Left DS Line
        Translation2d(119.938, 0.0) to Translation2d(119.938, 319.067), //Init line near
        Translation2d(121.938, 0.0) to Translation2d(121.938, 319.067), //Init line far
        Translation2d(206.557, 319.067) to Translation2d(206.557, 265.689), //Trench line near outer
        Translation2d(206.557, 265.689) to Translation2d(422.557, 265.689), //Trench line vert outer
        Translation2d(422.557, 265.689) to Translation2d(422.557, 319.067), //Trench line far outer
        Translation2d(208.557, 319.067) to Translation2d(208.557, 267.689), //Trench line near inner
        Translation2d(208.557, 267.689) to Translation2d(420.557, 267.689), //Trench line vert inner
        Translation2d(420.557, 267.689) to Translation2d(420.557, 319.067), //Trench line far inner
        Translation2d(344.578, 319.067) to Translation2d(344.578, 265.689), //Tunnel near
        Translation2d(374.578, 319.067) to Translation2d(374.578, 265.689), //Tunnel far
        Translation2d(216.631, 204.210) to Translation2d(207.392, 200.383), //Truss boundary 1
        Translation2d(206.861, 199.076) to Translation2d(210.688, 189.837), //Truss boundary 2
        Translation2d(0.0, 202.533) to Translation2d(30.0, 226.533), //Goal boundary 1
        Translation2d(30.0, 226.533) to Translation2d(0.0, 250.533) //Goal boundary 2
    )

    val controlPanelDiameter = 24.0.Inches


    val outerPortCenterHeight = 8.0.Feet.toInches() + 2.25.Inches


}