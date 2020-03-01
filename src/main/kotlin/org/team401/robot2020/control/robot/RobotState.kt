package org.team401.robot2020.control.robot

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.sevision.*
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.geometry.Twist2d
import org.team401.taxis.util.InterpolatingDouble
import org.team401.taxis.util.InterpolatingTreeMap
import kotlin.math.hypot

object RobotState: DifferentialDriveState(100, DrivetrainSubsystem.model.driveKinematicsModel) {
    private val vehicleToTurret = InterpolatingTreeMap<InterpolatingDouble, Pose2d>(observationBufferSize)
    var turretVelocityPredicted = Twist2d.identity()
        @Synchronized get
        private set

    private val cameraToGoal = InterpolatingTreeMap<InterpolatingDouble, Pose2d>(observationBufferSize)

    //private val visionTarget = GoalTracker()

    var lastSeenFieldToTarget = Pose2d.identity()

    private val cameraToVisionTargetTranslations = arrayListOf<Translation2d?>()

    init {
        cameraToGoal[InterpolatingDouble(0.0)] = Pose2d.identity()
    }

    @Synchronized fun resetVision() {
        //visionTarget.reset()
    }

    @Synchronized fun addTurretObservation(timestamp: TimeMeasureSeconds, turretAngle: AngularDistanceMeasureRadians, turretVelocity: AngularVelocityMeasureRadiansPerSecond) {
        val rotation = Rotation2d.fromRadians((turretAngle - ShooterConstants.turretZeroOffset).value)
        vehicleToTurret[InterpolatingDouble(timestamp.value)] = Pose2d(ShooterConstants.robotToTurret, rotation)
        turretVelocityPredicted = Twist2d(0.0, 0.0, turretVelocity.value)
    }

    private fun solveCameraToVisionTargetTranslation(target: TargetInfo, source: LimelightCamera): Translation2d? {
        val xzPlaneTranslation = Translation2d(target.x, target.z).rotateBy(source.horizontalPlaneToLens)
        val x = xzPlaneTranslation.x()
        val y = target.y
        val z = xzPlaneTranslation.y()

        val differentialHeight = source.lensHeight - 98.0 //TODO move magic number into constants (98 is height of goal)
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            val scaling = differentialHeight / -z
            val distance = hypot(x, y) * scaling
            val angle = Rotation2d(x, y, true)
            return Translation2d(distance * angle.cos(), distance * angle.sin())
        }

        return null
    }

    private fun updateGoalTracker(timestamp: TimeMeasureSeconds, source: LimelightCamera) {
        if (cameraToVisionTargetTranslations.size != 2 ||
                cameraToVisionTargetTranslations[0] == null ||
                cameraToVisionTargetTranslations[1] == null) return

        val cameraToVisionTarget = Pose2d.fromTranslation(
            cameraToVisionTargetTranslations[0]!!.interpolate(
                cameraToVisionTargetTranslations[1]!!, 0.5
            )
        )

        val fieldToVisionTarget = getFieldToTurret(timestamp) //Field -> Turret
            .transformBy(source.originToLens) //Turret -> Camera
            .transformBy(cameraToVisionTarget) //Camera -> Target

        lastSeenFieldToTarget = Pose2d(fieldToVisionTarget.translation, Rotation2d.identity())

        //visionTarget.update(timestamp.value, listOf(Pose2d(fieldToVisionTarget.translation, Rotation2d.identity())))
    }

    @Synchronized fun addVisionUpdate(timestamp: TimeMeasureSeconds, observations: List<TargetInfo>?, source: LimelightCamera) {
        cameraToVisionTargetTranslations.clear()

        //Empty update case
        if (observations == null || observations.isEmpty()) {
            //visionTarget.update(timestamp.value, arrayListOf())
            return
        }

        observations.forEach {
            target ->
            cameraToVisionTargetTranslations.add(solveCameraToVisionTargetTranslation(target, source))
        }

        updateGoalTracker(timestamp, source)
    }

    @Synchronized fun addCameraObservation(angleToGoal: Rotation2d, distanceToGoal: Double, time: TimeMeasureSeconds) {
        cameraToGoal[InterpolatingDouble(time.value)] = Pose2d(distanceToGoal, 0.0, angleToGoal)
    }

    @Synchronized fun getVehicleToTurret(timestamp: TimeMeasureSeconds): Pose2d {
        return vehicleToTurret.getInterpolated(InterpolatingDouble(timestamp.value))
    }

    @Synchronized fun getFieldToTurret(timestamp: TimeMeasureSeconds): Pose2d {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp))
    }

    @Synchronized fun getLatestVehicleToTurret(): Map.Entry<InterpolatingDouble, Pose2d> {
        return vehicleToTurret.lastEntry()
    }

    @Synchronized fun getLatestFieldToTurret(): Pose2d {
        return getLatestFieldToVehicle().value.transformBy(getLatestVehicleToTurret().value)
    }

    @Synchronized fun getPredictedVehicleToTurret(lookaheadTime: TimeMeasureSeconds): Pose2d {
        return getLatestVehicleToTurret().value
            .transformBy(Pose2d.exp(turretVelocityPredicted.scaled(lookaheadTime.value)))
    }

    @Synchronized fun getPredictedFieldToTurret(lookaheadTime: TimeMeasureSeconds): Pose2d {
        return getPredictedFieldToVehicle(lookaheadTime)
            .transformBy(getPredictedVehicleToTurret(lookaheadTime))
    }

    @Synchronized fun getFieldToVisionTarget(): Pose2d? {
        return Pose2d(lastSeenFieldToTarget.translation, Rotation2d.identity()) //We know the target is at 0 degrees
    }

    @Synchronized fun getVehicleToVisionTarget(timestamp: TimeMeasureSeconds): Pose2d? {
        val fieldToVisionTarget = getFieldToVisionTarget() ?: return null

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget)
    }

    @Synchronized fun getAimingParameters(timestamp: TimeMeasureSeconds, prevTrackId: Int, maxTrackAge: Double): AimingParameters? {
        val vehicleToGoal = getFieldToVehicle(timestamp)
            .inverse()
            .transformBy(lastSeenFieldToTarget)

        val turretToGoal = getFieldToTurret(timestamp)
            .inverse() // Turret -> Field
            .transformBy(lastSeenFieldToTarget) // Turret -> Target

        return AimingParameters(
            vehicleToGoal,
            turretToGoal,
            lastSeenFieldToTarget,
            lastSeenFieldToTarget.rotation,
            timestamp.value,
            0.0,
            0
        )
    }
}