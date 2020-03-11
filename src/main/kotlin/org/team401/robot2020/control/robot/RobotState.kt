package org.team401.robot2020.control.robot

import org.snakeskin.dsl.readTimestamp
import org.snakeskin.measure.Inches
import org.snakeskin.measure.RadiansPerSecond
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
    private val cameraToVisionTargetTranslations = arrayListOf<Translation2d?>()

    /**
     * The velocity of the turret.  This is used to predict future turret positions.
     */
    var turretVelocityPredicted = Twist2d.identity()
        @Synchronized get
        private set

    private var lastTargetingParameters: ShooterTargetingParameters? = null

    /**
     * The latest field to target reference.
     */
    var fieldToTargetLock = FieldToTargetLock(0.0.Seconds, Pose2d.identity())
        @Synchronized get
        private set

    @Synchronized fun resetVision() {
        fieldToTargetLock = FieldToTargetLock(readTimestamp(), getLatestFieldToVehicle().value.transformBy(Pose2d(-50.0, 0.0, Rotation2d.identity())))
    }

    init {
        vehicleToTurret[InterpolatingDouble(0.0)] = Pose2d(ShooterConstants.robotToTurret, Rotation2d.identity())
    }

    /**
     * Manually observes a new field to target reference.  Useful in auto to override the lock with a known location.
     */
    @Synchronized fun observeFieldToTarget(timestamp: TimeMeasureSeconds, fieldToTarget: Pose2d) {
        fieldToTargetLock = FieldToTargetLock(timestamp, fieldToTarget)
    }

    /**
     * Adds an observation from the turret, which consists of the time, turet angle, and turret velocity.
     */
    @Synchronized fun addTurretObservation(timestamp: TimeMeasureSeconds, turretAngle: AngularDistanceMeasureRadians, turretVelocity: AngularVelocityMeasureRadiansPerSecond) {
        val rotation = Rotation2d.fromRadians(turretAngle.value)
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
        //Ensure two proper corners
        if (cameraToVisionTargetTranslations.size != 2 ||
                cameraToVisionTargetTranslations[0] == null ||
                cameraToVisionTargetTranslations[1] == null) return

        //Interpolate center of the goal
        val cameraToVisionTarget = Pose2d.fromTranslation(
            cameraToVisionTargetTranslations[0]!!.interpolate(
                cameraToVisionTargetTranslations[1]!!, 0.5
            )
        )

        val fieldToVisionTarget = getFieldToTurret(timestamp) //Field -> Turret
            .transformBy(source.originToLens) //Turret -> Camera
            .transformBy(cameraToVisionTarget) //Camera -> Target

        fieldToTargetLock = FieldToTargetLock(timestamp, Pose2d(fieldToVisionTarget.translation, Rotation2d.identity()))
    }

    /**
     * Adds an update from a vision camera.
     */
    @Synchronized fun addVisionUpdate(timestamp: TimeMeasureSeconds, observations: List<TargetInfo>?, source: LimelightCamera) {
        cameraToVisionTargetTranslations.clear()

        //Empty update case
        if (observations == null || observations.isEmpty()) {
            return
        }

        observations.forEach {
            target ->
            cameraToVisionTargetTranslations.add(solveCameraToVisionTargetTranslation(target, source))
        }

        updateGoalTracker(timestamp, source)
    }

    /**
     * Returns the 2d transform from the vehicle to the turret.
     * The translation is a physical constant, the rotation is measured by the encoder
     */
    @Synchronized fun getVehicleToTurret(timestamp: TimeMeasureSeconds): Pose2d {
        return vehicleToTurret.getInterpolated(InterpolatingDouble(timestamp.value))
    }

    /**
     * Gets the pose of the turret on the field at the given timestamp
     */
    @Synchronized fun getFieldToTurret(timestamp: TimeMeasureSeconds): Pose2d {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp))
    }

    /**
     * Gets the latest timestamp and pose of the turret relative to the vehicle
     */
    @Synchronized fun getLatestVehicleToTurret(): Map.Entry<InterpolatingDouble, Pose2d> {
        return vehicleToTurret.lastEntry()
    }

    /**
     * Gets the latest pose of the turret relative to the field
     */
    @Synchronized fun getLatestFieldToTurret(): Pose2d {
        return getLatestFieldToVehicle().value.transformBy(getLatestVehicleToTurret().value)
    }

    /**
     * Predicts the position of the turret relative to the vehicle at the given number of seconds ahead of the latest observation
     */
    @Synchronized fun getPredictedVehicleToTurret(lookaheadTime: TimeMeasureSeconds): Pose2d {
        return getLatestVehicleToTurret().value
            .transformBy(Pose2d.exp(turretVelocityPredicted.scaled(lookaheadTime.value)))
    }

    /**
     * Predicts the position of the turret relative to the field at the given number of seconds ahead of the latest observation
     */
    @Synchronized fun getPredictedFieldToTurret(lookaheadTime: TimeMeasureSeconds): Pose2d {
        return getPredictedFieldToVehicle(lookaheadTime)
            .transformBy(getPredictedVehicleToTurret(lookaheadTime))
    }

    /**
     * Gets the latest pose of the vision target relative to the field
     */
    @Synchronized fun getFieldToVisionTarget(): Pose2d? {
        return Pose2d(fieldToTargetLock.fieldToTarget.translation, Rotation2d.identity()) //We know the target is at 0 degrees
    }

    /**
     * Gets the transform from the vehicle to the vision target at the given timestamp
     */
    @Synchronized fun getVehicleToVisionTarget(timestamp: TimeMeasureSeconds): Pose2d? {
        val fieldToVisionTarget = getFieldToVisionTarget() ?: return null

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget)
    }

    /**
     * Gets targeting parameters for the shooter at the given timestamp and lookahead prediction time.
     * Additionally caches the calculated parameters so that future calls with the same timestamp will not
     * need to recalculate the parameters.
     */
    @Synchronized fun getTargetingParameters(timestamp: TimeMeasureSeconds, lookaheadTime: TimeMeasureSeconds): ShooterTargetingParameters {
        if (lastTargetingParameters?.timestamp == timestamp) return lastTargetingParameters!! //Use cached to avoid recalculating

        val vehicleToGoal = getFieldToVehicle(timestamp)
            .inverse()
            .transformBy(fieldToTargetLock.fieldToTarget)

        val turretToGoal = getVehicleToTurret(timestamp)
            .inverse() //Turret to vehicle
            .transformBy(getFieldToVehicle(timestamp).inverse()) //Turret to field
            .transformBy(fieldToTargetLock.fieldToTarget) //Turret to target


        /*
        val turretToGoal = getFieldToTurret(timestamp)
            .inverse() // Turret -> Field
            .transformBy(fieldToTargetLock.fieldToTarget) // Turret -> Target

         */

        val vehicleToGoalDirection = vehicleToGoal.translation.direction() //Directions of vectors oriented at the target
        val turretToGoalDirection = vehicleToGoal.transformBy(Pose2d.fromTranslation(ShooterConstants.robotToTurret)).translation.direction()

        val vehicleToTurretNow = getVehicleToTurret(timestamp)

        val fieldToTurret = getFieldToTurret(timestamp)
        val fieldToPredictedTurret = getPredictedFieldToTurret(lookaheadTime)

        val turretToPredictedTurret = fieldToTurret
            .inverse() // turret -> field
            .transformBy(fieldToPredictedTurret) // turret -> predicted turret

        val predictedTurretToGoal = turretToPredictedTurret
            .inverse() // Predicted turret -> turret
            .transformBy(turretToGoal) // Predicted turret -> goal

        val correctedRangeToTarget = predictedTurretToGoal.translation.norm().Inches //Predicted range of the turret to the goal

        val turretError = vehicleToTurretNow.rotation //Relative rotation the turret must perform to align with the goal
            .inverse()
            .rotateBy(vehicleToGoalDirection)

        val vehicleRange = vehicleToGoal.translation.norm()
        val vehicleVelocity = vehicleVelocityMeasured
        val feedVelocity = (-1.0 * //Invert direction to have turret oppose robot motion
                ((vehicleToGoalDirection.sin() * vehicleVelocity.dx / vehicleRange) //Handle tangential movement of the robot around the goal
                        + vehicleVelocity.dtheta)).RadiansPerSecond //Handle rotational velocity of the robot

        val params = ShooterTargetingParameters(timestamp, turretError, feedVelocity, correctedRangeToTarget)
        lastTargetingParameters = params //Cache this value for future calls to avoid recalculating
        return params
    }
}