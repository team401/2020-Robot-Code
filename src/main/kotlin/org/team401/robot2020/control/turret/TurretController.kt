package org.team401.robot2020.control.turret

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.component.IAngularPositionVelocitySensorComponent
import org.snakeskin.component.provider.IPercentOutputMotorControlProvider
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.acceleration.angular.AngularAccelerationMeasureRadiansPerSecondPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.robot2020.util.getAcceleration
import org.team401.taxis.geometry.Rotation2d
import org.team401.util.PIDControllerLite
import org.team401.util.ProfiledPIDControllerLite
import org.team401.util.TrapezoidProfileLite

/**
 * Class implementing control logic for a multi-rotational turret.  Automatically handles different control modes
 * such as rapid moves, tracking, holding, and jogging.
 *
 * @param Kp The P gain for rapid moves and holding
 * @param Kd The D gain for rapid moves and holding
 * @param trackingKp The P gain for tracking moves
 * @param trackingKd The D gain for tracking moves
 * @param Ks The static friction feed forward coefficient
 * @param Kv The velocity feed forward coefficient
 * @param Ka The acceleration feed forward coefficient
 * @param velocityConstraint The velocity constraint for rapid moves
 * @param accelerationConstraint The acceleration constraint for rapid moves
 * @param rapidThresholdDistance The minimum required rotation which will trigger a rapid move
 * @param lowerLimitAngleAbsolute The absolute lower travel limit of the turret, as measured by the encoder
 * @param upperLimitAngleAbsolute The absolute upper travel limit of the turret, as measured by the encoder
 * @param inputProvider The provider for input data about the turret (position, velocity)
 * @param outputProvider The provider for output signals of the turret (percent output control)
 * @param rate The rate the controller will be updated at, in seconds
 */
class TurretController(
    private val Kp: Double,
    private val Kd: Double,
    private val trackingKp: Double,
    private val trackingKd: Double,

    private val Ks: Double,
    private val Kv: Double,
    private val Ka: Double,

    private val velocityConstraint: AngularVelocityMeasureRadiansPerSecond,
    private val accelerationConstraint: AngularAccelerationMeasureRadiansPerSecondPerSecond,

    private val rapidThresholdDistance: AngularDistanceMeasureRadians,

    private val lowerLimitAngleAbsolute: AngularDistanceMeasureRadians,
    private val upperLimitAngleAbsolute: AngularDistanceMeasureRadians,

    private val inputProvider: IAngularPositionVelocitySensorComponent,
    private val outputProvider: IPercentOutputMotorControlProvider,
    private val rate: TimeMeasureSeconds
) {
    //Controllers and model
    private val rapidConstraints = TrapezoidProfileLite.Constraints(velocityConstraint.value, accelerationConstraint.value)
    private val rapidController = ProfiledPIDControllerLite(Kp, 0.0, Kd, rapidConstraints, rate.value)
    private var lastRapidState = TrapezoidProfileLite.State()
    private val holdController = PIDControllerLite(Kp, 0.0, Kd, rate.value)
    private val trackingController = PIDControllerLite(trackingKp, 0.0, trackingKd, rate.value)
    private val model = SimpleMotorFeedforward(Ks, Kv, Ka)

    //State info
    private var currentPosition = 0.0.Radians
    private var currentVelocity = 0.0.RadiansPerSecond

    init {
        rapidController.setTolerance(Double.POSITIVE_INFINITY)
    }

    //Limit checking functions.  Return true for a safe angle, false for an unsafe angle
    private fun checkLowerLimit(angle: AngularDistanceMeasureRadians) = (angle >= lowerLimitAngleAbsolute)
    private fun checkUpperLimit(angle: AngularDistanceMeasureRadians) = (angle <= upperLimitAngleAbsolute)
    private fun checkLimits(angle: AngularDistanceMeasureRadians) = checkLowerLimit(angle) && checkUpperLimit(angle)

    //Handles turret wraparound and shortest distance tracking using last known position
    private fun wrapAngle(targetRotationIn: Rotation2d): AngularDistanceMeasureRadians {
        val targetRotation = Rotation2d(targetRotationIn.radians, true) //Clamp target to -180..180
        val currentRotation = Rotation2d(currentPosition.value, true) //Clamp current position to -180..180
        val delta = currentRotation.inverse().rotateBy(targetRotation) //Solve distance to go

        var targetPosition = currentPosition + delta.radians.Radians //Attempt shortest route
        if (!checkUpperLimit(targetPosition)) {
            targetPosition -= 360.0.Degrees.toRadians() //Turret is unsafe in positive, shortest route is reverse
        }
        if (!checkLowerLimit(targetPosition)) {
            targetPosition += 360.0.Degrees.toRadians() //Turret tis unsafe in negative, shortest route is forward
        }

        return targetPosition //This is now guaranteed to be clamped safely
    }

    //Checks if a move should be a rapid
    private fun shouldRapid(targetPosition: AngularDistanceMeasureRadians): Boolean {
        return (targetPosition - currentPosition).abs() > rapidThresholdDistance
    }

    private enum class ControlMode {
        Angle,
        Jog,
        Hold,
        AbsoluteAngle
    }

    private var currentControlMode = ControlMode.Hold

    private fun updateState() {
        //Read inputs from the turret
        currentPosition = inputProvider.getAngularPosition()
        currentVelocity = inputProvider.getAngularVelocity()
    }

    private fun updateTracking(goal: Rotation2d, feedVelocity: AngularVelocityMeasureRadiansPerSecond) {
        val angle = wrapAngle(goal)
        println(angle.toDegrees())

        if (shouldRapid(angle)) {
           //We need to rapid to the angle
            updateRapid(angle)
        } else {
            //No rapid, standard tracking
            isInRapid = false

            val feedbackVolts = trackingController.calculate(currentPosition.value, angle.value)
            val ffVolts = model.calculate(feedVelocity.value)

            outputProvider.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
        }
    }

    //Rapid
    var isInRapid = false
        private set

    private fun updateRapid(goal: AngularDistanceMeasureRadians) {
        if (!isInRapid) {
            //Rapid is starting
            lastRapidState = TrapezoidProfileLite.State(currentPosition.value, currentVelocity.value)
            rapidController.reset(lastRapidState)
            rapidController.setGoal(goal.value)
            isInRapid = true
        }
        //Update the rapid
        val feedbackVolts = rapidController.calculate(currentPosition.value, goal.value)
        val ffVolts = model.calculate(
            rapidController.setpoint.velocity,
            rapidController.getAcceleration(lastRapidState)
        )

        lastRapidState = rapidController.setpoint
        outputProvider.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
    }

    //Hold mode
    fun enterHold() {
        updateState()
        holdController.reset()
        holdController.setpoint = currentPosition.value
        currentControlMode = ControlMode.Hold
        isInRapid = false
    }

    fun updateHold() {
        if (currentControlMode != ControlMode.Hold) return

        updateState()
        val feedbackVolts = holdController.calculate(currentPosition.value)
        outputProvider.setPercentOutput(feedbackVolts / 12.0)
    }

    //Jog mode
    private var jogSetpoint = Rotation2d.identity()

    fun enterJog() {
        updateState()
        trackingController.reset()
        jogSetpoint = Rotation2d.fromRadians(currentPosition.value)
        currentControlMode = ControlMode.Jog
    }

    fun updateJog(rate: AngularVelocityMeasureRadiansPerSecond, dt: TimeMeasureSeconds) {
        if (currentControlMode != ControlMode.Jog) return

        updateState()
        val jogAdvance = Rotation2d.fromRadians((rate * dt).value)
        jogSetpoint = jogSetpoint.rotateBy(jogAdvance)

        updateTracking(jogSetpoint, 0.0.RadiansPerSecond)
    }

    fun enterAngle() {
        updateState()
        trackingController.reset()
        currentControlMode = ControlMode.Angle
    }

    fun updateAngle(angleGoal: Rotation2d, feedVelocity: AngularVelocityMeasureRadiansPerSecond = 0.0.RadiansPerSecond) {
        if (currentControlMode != ControlMode.Angle) return

        updateState()
        updateTracking(angleGoal, feedVelocity)
    }

    fun enterAbsoluteAngle() {
        updateState()
        lastRapidState = TrapezoidProfileLite.State(currentPosition.value, currentVelocity.value)
        rapidController.reset(lastRapidState)
        currentControlMode = ControlMode.AbsoluteAngle
    }

    fun updateAbsoluteAngle(angleAbsolute: AngularDistanceMeasureRadians) {
        updateState()
        val feedbackVolts = rapidController.calculate(currentPosition.value, angleAbsolute.value)
        val ffVolts = model.calculate(
            rapidController.setpoint.velocity,
            rapidController.getAcceleration(lastRapidState)
        )
        lastRapidState = rapidController.setpoint

        outputProvider.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
    }
}