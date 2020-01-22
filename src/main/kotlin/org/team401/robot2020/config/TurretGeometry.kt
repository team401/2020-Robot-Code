package org.team401.robot2020.config

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.team401.taxis.geometry.Translation2d

object TurretGeometry {
    val robotToTurretTranslation = Translation2d(16.375, 0.0)

    val turretCameraMountingAngle = 15.0.Degrees
    val turretCameraMountingHeight = 5.0.Inches
}