package org.team401.robot2020.config

import org.snakeskin.measure.FeetPerSecond
import org.snakeskin.measure._lbmass
import org.snakeskin.measure._rev_per_s
import org.snakeskin.measure._rev_per_s_per_s
import org.team401.taxis.template.DifferentialDrivetrainDynamicsParameters

object DrivetrainDynamics: DifferentialDrivetrainDynamicsParameters {
    override val angularDrag = 0.0
    override val inertialMass = 46.75

    override val leftKa = 0.0172
    override val leftKs = 0.216
    override val leftKv = 0.186
    override val momentOfInertia = 4.779
    override val rightKa = 0.0163
    override val rightKs = 0.341
    override val rightKv = 0.187
    override val trackScrubFactor = 1.0020927384804579

    val driveSpeedIdeal = 15.7.FeetPerSecond

    val driveLeftKp = 1.62
    val driveRightKp = 1.53
}