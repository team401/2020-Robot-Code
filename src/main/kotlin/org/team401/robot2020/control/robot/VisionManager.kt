package org.team401.robot2020.control.robot

import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.subsystems.ShooterSubsystem
import org.team401.sevision.LimelightCamera

object VisionManager: RealTimeTask() {
    private var turretVisionActive = false

    val turretLimelight = LimelightCamera(
        "turret",
        ShooterConstants.turretCameraNearOriginToLens,
        ShooterConstants.turretCameraNearLensHeight.value,
        ShooterConstants.turretCameraNearHorizontalPlaneToLens
    )

    private const val nearPipeline = 1
    private const val farPipeline = 2

    @Synchronized override fun action(timestamp: TimeMeasureSeconds, dt: TimeMeasureSeconds) {
        if (turretVisionActive && !ShooterSubsystem.isTurretInRapid()) { //Do not target if the turret is in rapid
            RobotState.addVisionUpdate(timestamp - turretLimelight.latency.Seconds, turretLimelight.target, turretLimelight)
        } else {
            RobotState.addVisionUpdate(timestamp - turretLimelight.latency.Seconds, null, turretLimelight)
        }
    }

    /**
     * Turns off the turret camera
     */
    @Synchronized fun turretVisionOff() {
        turretVisionActive = false
        turretLimelight.setLedOff()
        turretLimelight.setPipeline(nearPipeline)
    }

    /**
     * Configures the turret for near targeting
     */
    @Synchronized fun turretVisionNearTargeting() {
        turretVisionActive = true
        turretLimelight.setPipeline(nearPipeline)
        turretLimelight.markPipeline1xZoom()
        turretLimelight.setLedToPipeline()
        turretLimelight.originToLens = ShooterConstants.turretCameraNearOriginToLens
        turretLimelight.lensHeight = ShooterConstants.turretCameraNearLensHeight.value
        turretLimelight.horizontalPlaneToLens = ShooterConstants.turretCameraNearHorizontalPlaneToLens
    }

    /**
     * Configures the turret for far targeting
     */
    @Synchronized fun turretVisionFarTargeting() {
        turretVisionActive = true
        turretLimelight.setPipeline(farPipeline)
        turretLimelight.markPipeline2xZoom()
        turretLimelight.setLedToPipeline()
        turretLimelight.originToLens = ShooterConstants.turretCameraFarOriginToLens
        turretLimelight.lensHeight = ShooterConstants.turretCameraFarLensHeight.value
        turretLimelight.horizontalPlaneToLens = ShooterConstants.turretCameraFarHorizontalPlaneToLens
    }
}