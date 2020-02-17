package org.team401.robot2020.subsystems

import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.Gearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Unitless
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.constants.ClimbingConstants
import kotlin.math.tan

object ClimbingSubsystem : Subsystem() {

    private val climbLeftElevatorMotor = Hardware.createBrushlessSparkMax(CANDevices.climbLeftElevatorMotor.canID)
    private val climbRightElevatorMotor = Hardware.createBrushlessSparkMax(CANDevices.climbRightElevatorMotor.canID)

    private val climbLeftElevatorEncoder = Hardware.createCANCoder(CANDevices.climbLeftElevatorEncoder)
    private val climbRightElevatorEncoder = Hardware.createCANCoder(CANDevices.climbRightElevatorEncoder)

    private val leftClimber = Gearbox(climbLeftElevatorEncoder, climbLeftElevatorMotor)
    private val rightClimber = Gearbox(climbRightElevatorEncoder, climbRightElevatorMotor)

    private val imu = Hardware.createCANPigeonIMU(CANDevices.pigeon)

    private val pid = ProfiledPIDController(
        ClimbingConstants.kP,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(ClimbingConstants.maxVelocity.value, ClimbingConstants.maxAccel.value),
        .01
    )

    private val model = SimpleMotorFeedforward(0.0, 0.0)

    var leftClimberPosition = leftClimber.getAngularPosition().toDegrees()
    var rightClimberPosition = rightClimber.getAngularPosition().toDegrees()

    private fun resetClimberPosition() {
        leftClimberPosition = 0.0.Degrees
        rightClimberPosition = 0.0.Degrees
    }

    private fun calculateAngularMovement(inches: LinearDistanceMeasureInches): AngularDistanceMeasureDegrees {
        return ((inches.value * 360.0) / (ClimbingConstants.sprocketPitchDiameter.value * 3.14159)).Degrees
    }

    private fun calculateHangingOffset(angle: AngularDistanceMeasureDegrees): LinearDistanceMeasureInches {
        return (ClimbingConstants.distBetweeHooks * tan(angle.value).Unitless).toInches()
    }

    private fun calculateMotorPower(climberPosition: AngularDistanceMeasureDegrees, targetClimberPosition: AngularDistanceMeasureDegrees): Double {
        val feedback = pid.calculate(climberPosition.value, targetClimberPosition.value)
        val outputVelocity = pid.setpoint.velocity
        val feedForward = model.calculate(outputVelocity)
        val power = (feedForward + feedback) / 12.0
        return power
    }

    enum class ClimbingStates {
        Extended,
        Hanging,
        Retracted
    }

    val ClimbingMachine : StateMachine<ClimbingStates> = stateMachine {

        state(ClimbingStates.Extended) {
            entry {
                //resets with the average position of left and right climber
                //TODO: Remove?
                pid.reset((leftClimberPosition + rightClimberPosition).value / 2)
            }

            action {
                leftClimberPosition = leftClimber.getAngularPosition().toDegrees()
                rightClimberPosition = rightClimber.getAngularPosition().toDegrees()

                val targetLeftPosition = calculateAngularMovement(24.0.Inches)
                val targetRightPosition = calculateAngularMovement(24.0.Inches)

                val leftPower = calculateMotorPower(leftClimberPosition, targetLeftPosition)
                val rightPower = calculateMotorPower(rightClimberPosition, targetRightPosition)

                leftClimber.setPercentOutput(leftPower)
                rightClimber.setPercentOutput(rightPower)
            }
        }

        state(ClimbingStates.Hanging) {
            entry {
                pid.reset((leftClimberPosition + rightClimberPosition).value / 2)
            }

            action {
                leftClimberPosition = leftClimber.getAngularPosition().toDegrees()
                rightClimberPosition = rightClimber.getAngularPosition().toDegrees()

                val pigeonAngle = DrivetrainSubsystem.getRoll()

                var targetLeftPosition = calculateAngularMovement(10.0.Inches)
                var targetRightPosition = calculateAngularMovement(10.0.Inches)

                if (pigeonAngle.value < 0) {
                    targetRightPosition += calculateAngularMovement(calculateHangingOffset(pigeonAngle))
                } else {
                    targetLeftPosition += calculateAngularMovement(calculateHangingOffset(pigeonAngle))
                }

                val leftPower = calculateMotorPower(leftClimberPosition, targetLeftPosition)
                val rightPower = calculateMotorPower(rightClimberPosition, targetRightPosition)

                leftClimber.setPercentOutput(leftPower)
                rightClimber.setPercentOutput(rightPower)
            }
        }

        state(ClimbingStates.Retracted) {
            entry {
                pid.reset((leftClimberPosition + rightClimberPosition).value / 2)
            }

            action {
                leftClimberPosition = leftClimber.getAngularPosition().toDegrees()
                rightClimberPosition = rightClimber.getAngularPosition().toDegrees()

                val targetLeftPosition = calculateAngularMovement(0.0.Inches)
                val targetRightPosition = calculateAngularMovement(0.0.Inches)

                val leftPower = calculateMotorPower(leftClimberPosition, targetLeftPosition)
                val rightPower = calculateMotorPower(rightClimberPosition, targetRightPosition)

                leftClimber.setPercentOutput(leftPower)
                rightClimber.setPercentOutput(rightPower)
            }
        }
    }

    override fun setup() {
        useHardware(climbRightElevatorMotor) {
            enableVoltageCompensation(12.0)
        }

        useHardware(climbLeftElevatorMotor) {
            enableVoltageCompensation(12.0)
        }

        on(Events.TELEOP_ENABLED) {
            resetClimberPosition()
            ClimbingMachine.setState(ClimbingStates.Retracted)

        }
    }
}