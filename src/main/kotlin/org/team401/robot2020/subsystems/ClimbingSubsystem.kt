package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.Gearbox
import org.snakeskin.component.LinearTransmission
import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.NullCanCoderDevice
import org.snakeskin.component.impl.NullPneumaticChannel
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.component.impl.NullTalonSrxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.runtime.SnakeskinRuntime
import org.snakeskin.utility.Ticker
import org.snakeskin.utility.value.AsyncBoolean
import org.snakeskin.utility.value.AsyncValue
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.PneumaticDevices
import org.team401.robot2020.config.constants.ClimbingConstants
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.util.CharacterizationRoutine
import org.team401.robot2020.util.getAcceleration
import kotlin.math.tan
import kotlin.time.milliseconds

object ClimbingSubsystem : Subsystem() {
    //<editor-fold desc="Hardware Devices">
    private val climbLeftElevatorMotor = Hardware.createTalonSRX(
        CANDevices.climbLeftElevatorMotor.canID,
        mockProducer = NullTalonSrxDevice.producer
    )
    private val climbRightElevatorMotor = Hardware.createTalonSRX(
        CANDevices.climbRightElevatorMotor.canID,
        mockProducer = NullTalonSrxDevice.producer
    )


    private val lockingPiston= Hardware.createPneumaticChannel(
        PneumaticDevices.climbingLock,
        halMock = true
    )

    private val leftElevator = LinearTransmission(SmartGearbox(climbLeftElevatorMotor), ClimbingConstants.pitchRadius)
    private val rightElevator = LinearTransmission(SmartGearbox(climbRightElevatorMotor), ClimbingConstants.pitchRadius)
    //</editor-fold>

    //<editor-fold desc="Models and Controllers">
    private val leftElevatorModel = ElevatorFeedforward(
        ClimbingConstants.leftElevatorKs,
        ClimbingConstants.leftElevatorKg,
        ClimbingConstants.leftElevatorKv,
        ClimbingConstants.leftElevatorKa
    )

    private val rightElevatorModel = ElevatorFeedforward(
        ClimbingConstants.rightElevatorKs,
        ClimbingConstants.rightElevatorKg,
        ClimbingConstants.rightElevatorKv,
        ClimbingConstants.rightElevatorKa
    )

    private val elevatorConstraints = TrapezoidProfile.Constraints(
        ClimbingConstants.maxAngularVelocity.value,
        ClimbingConstants.maxAngularAccel.value
    )
    
    private val leftElevatorController = ProfiledPIDController(
        ClimbingConstants.leftElevatorKp,
        0.0, 0.0,
        elevatorConstraints,
        RobotConstants.rtPeriod.value
    )

    private val rightElevatorController = ProfiledPIDController(
        ClimbingConstants.rightElevatorKp,
        0.0, 0.0,
        elevatorConstraints,
        RobotConstants.rtPeriod.value
    )

    private var lastLeftElevatorSetpoint = TrapezoidProfile.State()
    private var lastRightElevatorSetpoint = TrapezoidProfile.State()

    private val leftElevatorJogController = PIDController(ClimbingConstants.leftElevatorKp, 0.0, 0.0)
    private val rightElevatorJogController = PIDController(ClimbingConstants.rightElevatorKp, 0.0, 0.0)

    val leftElevatorCharacterizationRoutine = CharacterizationRoutine(leftElevator.gearbox, leftElevator.gearbox)
    val rightElevatorCharacterizationRoutine = CharacterizationRoutine(rightElevator.gearbox, rightElevator.gearbox)
    //</editor-fold>

    private fun resetClimberPosition() {
        leftElevator.setLinearPosition(0.0.Inches)
        rightElevator.setLinearPosition(0.0.Inches)
    }

    /**
     * Updates the profiled controller for the left climb elevator
     */
    private fun updateLeftElevator(target: LinearDistanceMeasureInches) {
        val angularTarget = target.toAngularDistance(ClimbingConstants.pitchRadius)
        
        val feedbackVolts = leftElevatorController.calculate(leftElevator.gearbox.getAngularPosition().toRadians().value, angularTarget.value)
        val ffVolts = leftElevatorModel.calculate(
            leftElevatorController.setpoint.velocity,
            0.0//leftElevatorController.getAcceleration(elevatorConstraints, lastLeftElevatorSetpoint)
        )
        lastLeftElevatorSetpoint = leftElevatorController.setpoint

        leftElevator.gearbox.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
    }

    /**
     * Updates the profiled controller for the right climb elevator
     */
    private fun updateRightElevator(target: LinearDistanceMeasureInches) {
        val angularTarget = target.toAngularDistance(ClimbingConstants.pitchRadius)

        val feedbackVolts = rightElevatorController.calculate(rightElevator.gearbox.getAngularPosition().toRadians().value, angularTarget.value)
        val ffVolts = rightElevatorModel.calculate(
            rightElevatorController.setpoint.velocity,
            0.0//rightElevatorController.getAcceleration(elevatorConstraints, lastRightElevatorSetpoint)
        )
        lastRightElevatorSetpoint = rightElevatorController.setpoint

        rightElevator.gearbox.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
    }

    /**
     * Updates both profiled controllers simultaneously
     */
    private fun update(leftHeight: LinearDistanceMeasureInches, rightHeight: LinearDistanceMeasureInches = leftHeight) {
        updateLeftElevator(leftHeight)
        updateRightElevator(rightHeight)
    }

    /**
     * Resets the profiled controllers
     */
    private fun resetProfiles() {
        leftElevatorController.reset(leftElevator.gearbox.getAngularPosition().toRadians().value)
        rightElevatorController.reset(rightElevator.gearbox.getAngularPosition().toRadians().value)
        lastLeftElevatorSetpoint = TrapezoidProfile.State()
        lastRightElevatorSetpoint = TrapezoidProfile.State()
    }

    /**
     * Returns the lowest position of the two elevators
     */
    private fun getMinPosition() = leftElevator.getLinearPosition().coerceAtMost(rightElevator.getLinearPosition())

    private var jogPosition = ClimbingConstants.deployHeight

    /**
     * Resets the jog controllers
     */
    private fun resetJog() {
        leftElevatorJogController.reset()
        rightElevatorJogController.reset()
        jogPosition = getMinPosition()
    }

    /**
     * Updates the jog controllers using a direction
     */
    private fun updateJog(direction: MeasureUnitless) {
        var velocity = (ClimbingConstants.jogRate * direction).toAngularVelocity(ClimbingConstants.pitchRadius)
        if (jogPosition >= ClimbingConstants.maxHeight) {
            jogPosition = ClimbingConstants.maxHeight
            velocity = 0.0.RadiansPerSecond
        }
        if (jogPosition <= ClimbingConstants.minHeight) {
            jogPosition = ClimbingConstants.minHeight
            velocity = 0.0.RadiansPerSecond
        }

        val angularTarget = jogPosition.toAngularDistance(ClimbingConstants.pitchRadius)

        val leftFeedbackVolts = leftElevatorJogController.calculate(leftElevator.gearbox.getAngularPosition().toRadians().value, angularTarget.value)
        val leftFfVolts = leftElevatorModel.calculate(velocity.value)
        val rightFeedbackVolts = rightElevatorJogController.calculate(rightElevator.gearbox.getAngularPosition().toRadians().value, angularTarget.value)
        val rightFfVolts = rightElevatorModel.calculate(velocity.value)

        leftElevator.gearbox.setPercentOutput((leftFeedbackVolts + leftFfVolts) / 12.0)
        rightElevator.gearbox.setPercentOutput((rightFeedbackVolts + rightFfVolts) / 12.0)
    }

    private fun lock() {
        lockingPiston.setState(false)
    }

    private fun unlock() {
        lockingPiston.setState(true)
    }

    var homed by AsyncBoolean(false)
        private set

    var deployAttempted by AsyncBoolean(false)
        private set

    enum class ClimbingStates {
        Homing,
        Deploying,
        JogUp,
        JogDown,
        Hold,
        FixedClimb,
        Locking,
        Locked
    }

    val climbingMachine: StateMachine<ClimbingStates> = stateMachine {
        state(ClimbingStates.Homing) {
            val homingTicker = Ticker(
                {
                    leftElevator.gearbox.getAngularVelocity().abs() <= ClimbingConstants.homingVelocityThreshold &&
                    rightElevator.gearbox.getAngularVelocity().abs() <= ClimbingConstants.homingVelocityThreshold
                },
                ClimbingConstants.homingTime,
                0.02.Seconds
            )

            entry {
                homed = false
                unlock()
                delay(ClimbingConstants.lockPistonDelay)
                homingTicker.reset()
            }

            action {
                homingTicker.check {
                    resetClimberPosition()
                    homed = true
                    disable()
                }
            }
        }

        /**
         * Deploys the hooks for climbing
         */
        state(ClimbingStates.Deploying) {
            rejectIf { !homed }

            entry {
                deployAttempted = true
                unlock()
                resetProfiles()
            }

            rtAction { timestamp, dt ->
                update(ClimbingConstants.deployHeight)
                val deployed = leftElevatorController.atGoal() && rightElevatorController.atGoal()
                if (deployed) setState(ClimbingStates.Hold)
            }
        }

        /**
         * Jogs the climbers upwards
         */
        state(ClimbingStates.JogUp) {
            rejectIf { !deployAttempted }

            entry {
                unlock()
                resetJog()
            }

            rtAction { timestamp, dt ->
                jogPosition += ClimbingConstants.jogRate * dt
                updateJog(1.0._ul)
            }
        }

        /**
         * Jogs the climbers downwards
         */
        state(ClimbingStates.JogDown) {
            rejectIf { !deployAttempted }

            entry {
                unlock()
                resetJog()
            }

            rtAction { timestamp, dt ->
                jogPosition -= ClimbingConstants.jogRate * dt
                updateJog((-1.0)._ul)
            }
        }

        /**
         * Holds the climbers at a steady position
         */
        state(ClimbingStates.Hold) {
            rejectIf { !deployAttempted }

            entry {
                unlock()
                resetJog()
            }

            rtAction { timestamp, dt ->
                updateJog(0.0._ul)
            }
        }

        /**
         * Brings the climbers downward in order to climb onto the bar
         */
        state(ClimbingStates.FixedClimb) {
            rejectIf { !homed || !deployAttempted }

            entry {
                unlock()
                resetProfiles()
            }

            rtAction { timestamp, dt ->
                update(ClimbingConstants.climbingHeight)
            }
        }

        /**
         * Prepares for locking of the climbing mechanism
         */
        state(ClimbingStates.Locking) {
            timeout(ClimbingConstants.lockPistonDelay, ClimbingStates.Locked)

            entry {
                lock()
                resetJog()
            }

            rtAction { timestamp, dt ->
                updateJog(0.0._ul)
            }
        }

        /**
         * Locks the climbing mechanism
         */
        state(ClimbingStates.Locked) {
            entry {
                lock()
            }

            action {
                leftElevator.gearbox.stop()
                rightElevator.gearbox.stop()
            }
        }

        disabled {
            entry {
                lock()
            }

            action {
                leftElevator.gearbox.stop()
                rightElevator.gearbox.stop()
            }
        }
    }

    override fun setup() {
        leftElevatorController.setTolerance(ClimbingConstants.positionTolerance.toAngularDistance(ClimbingConstants.pitchRadius).value)
        rightElevatorController.setTolerance(ClimbingConstants.positionTolerance.toAngularDistance(ClimbingConstants.pitchRadius).value)

        leftElevator.gearbox.invertInput(true)
        leftElevator.gearbox.invert(false)

        rightElevator.gearbox.invertInput(false)
        rightElevator.gearbox.invert(true)

        useHardware(climbRightElevatorMotor) {
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0)
            setNeutralMode(NeutralMode.Brake)
        }

        useHardware(climbLeftElevatorMotor) {
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0)
            setNeutralMode(NeutralMode.Brake)
        }

        resetClimberPosition()

        on(Events.ENABLED) {
            homed = true
            climbingMachine.disable()
            if (!homed) {
                climbingMachine.setState(ClimbingStates.Homing)
            } else {
                climbingMachine.disable()
            }
        }
    }
}