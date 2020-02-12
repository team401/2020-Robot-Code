package org.team401.robot2020.config

object HardwareMap {
    object DrivetrainMap {
        const val leftFrontFalconId = 0
        const val leftRearFalconId = 1
        const val rightFrontFaconId = 2
        const val rightRearFalconId = 3
        const val pigeonId = 0

        const val leftEncoderA = 0
        const val leftEncoderB = 1
        const val rightEncoderA = 2
        const val rightEncoderB = 3
    }

    object TurretMap {
        const val turretRotationId = 23

        const val turretEncoderA = 4
        const val turretEncoderB = 5
    }

    object BallMap {
        const val feederId = 3

        const val bottomGate = 6
        const val topGate = 7
    }
}