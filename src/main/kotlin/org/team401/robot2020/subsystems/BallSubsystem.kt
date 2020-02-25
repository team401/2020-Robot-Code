package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.ArmFeedforward
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.component.impl.NullTalonSrxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2020.config.constants.BallConstants
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.DIOChannels
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.util.CharacterizationRoutine
import org.team401.robot2020.util.inverted

/**
 * Ball handling subsystem, including the intake, flying V, and tower components.
 */
object BallSubsystem : Subsystem() {
    //<editor-fold desc="Hardware Devices">
    private val flyingVMotors = Hardware.createTalonSRX(
        CANDevices.flyingVMotors.canID,
        mockProducer = NullTalonSrxDevice.producer
    )
    private val towerMotor = Hardware.createBrushlessSparkMax(
        CANDevices.towerMotor.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val intakeWheelsMotor = Hardware.createBrushlessSparkMax(
        CANDevices.intakeWheelsMotor.canID,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val intakeArmMotor = Hardware.createBrushlessSparkMaxWithEncoder(
        CANDevices.intakePivotMotor.canID,
        8192, //REV Through Bore Encoder
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val intakeArmGearbox = SmartGearbox(intakeArmMotor)

    private val bottomGateSensor = Hardware.createDigitalInputChannel(
        DIOChannels.towerBottomGateSensor,
        halMock = true
    ).inverted()

    private val topGateSensor = Hardware.createDigitalInputChannel(
        DIOChannels.towerTopGateSensor,
        halMock = true
    ).inverted()
    //</editor-fold>

    //<editor-fold desc="Models and Controllers">
    private val intakeArmModel = ArmFeedforward(
        BallConstants.intakeArmKs,
        BallConstants.intakeArmKcos,
        BallConstants.intakeArmKv
    )

    private val intakeArmConstraints = TrapezoidProfile.Constraints(
        BallConstants.intakeArmVelocity.value,
        BallConstants.intakeArmAcceleration.value
    )
    private val intakeArmController = ProfiledPIDController(
        BallConstants.intakeArmKp, 0.0, 0.0,
        intakeArmConstraints,
        RobotConstants.rtPeriod.value
    )

    val armCharacterizationRoutine = CharacterizationRoutine(intakeArmGearbox, intakeArmGearbox)
    //</editor-fold>

    private fun resetIntakeArm() {
        intakeArmController.reset(intakeArmGearbox.getAngularPosition().toRadians().value)
    }

    private fun updateIntakeArm(angle: AngularDistanceMeasureRadians) {
        val position = intakeArmGearbox.getAngularPosition().toRadians().value
        val feedbackVolts = intakeArmController.calculate(position, angle.value)
        val ffVolts = intakeArmModel.calculate(intakeArmController.setpoint.position, intakeArmController.setpoint.velocity)

        val out = (feedbackVolts + ffVolts) / 12.0

        intakeArmGearbox.setPercentOutput(out)
    }

    enum class TowerStates {
        Waiting, //Default state, waiting for ball to trip the bottom gate
        Feeding, //Ball tripped gate, waiting for ball to clear bottom gate
        Spacing, //Timed state to advance ball forward to ensure proper spacing of the balls
        Reversing, //Top gate is on and bottom gate is off, reversing balls to ensure future spacing
        Full, //Tower is full (both top and bottom gate are on)
        Shooting, //Tower is feeding balls into the shooter
        ManualReverse //Tower is manually reversing balls
    }

    enum class FlyingVStates {
        Intaking, //State of the V when the robot is intaking
        Shooting, //State of the V when the robot is shooting
        Idle, //State of the V when the robot has stopped intaking.  Checks for tower feed over a timeout
        ManualReverse //Manually reverse the V
    }

    enum class IntakeStates {
        Stowed,
        Intake,
        GoToStow
    }

    val towerMachine: StateMachine<TowerStates> = stateMachine {
        state(TowerStates.Waiting) {
            action {
                towerMotor.stop() //No motion during waiting

                val bottomState = bottomGateSensor.getState()
                val topState = topGateSensor.getState()

                when {
                    (topState && bottomState) -> setState(TowerStates.Full) //Both gates triggered, tower is full
                    (topState && !bottomState) -> setState(TowerStates.Reversing) //Balls are not at the start of the tower, reverse
                    (bottomState) -> setState(TowerStates.Feeding) //Ball is ready to enter the tower
                }
            }
        }

        state(TowerStates.Feeding) {
            entry {
                //flyingVMachine.setState(FlyingVStates.Reverse)
            }

            action {
                towerMotor.setAngularVelocitySetpoint(BallConstants.towerFeedingRate.toRadiansPerSecond())

                val bottomState = bottomGateSensor.getState()
                val topState = topGateSensor.getState()

                when {
                    (topState && bottomState) -> setState(TowerStates.Full) //Both gates triggered, tower is full
                    (topState) -> setState(TowerStates.Reversing) //Spacing is somehow off and a new ball is being entered
                    (!bottomState) -> setState(TowerStates.Waiting) //New ball has entered the system
                }
            }
        }

        state(TowerStates.Spacing) {
            timeout(BallConstants.towerSpacingTime, TowerStates.Waiting) //Timed state to go back into waiting

            action {
                towerMotor.setPercentOutput(BallConstants.towerSpacingPower)
            }
        }

        state(TowerStates.Reversing) {
            entry {
                //flyingVMachine.setState(FlyingVStates.Reverse)
            }

            action {
                towerMotor.setPercentOutput(BallConstants.towerReversingPower)

                val bottomState = bottomGateSensor.getState()

                when {
                    (bottomState) -> setState(TowerStates.Spacing)
                }
            }
        }

        state(TowerStates.Full) {
            entry {
                //flyingVMachine.disable()
            }

            action {
                towerMotor.stop()

                val bottomState = bottomGateSensor.getState()
                val topState = topGateSensor.getState()

                if (!bottomState || !topState) setState(TowerStates.Waiting)
            }
        }

        state(TowerStates.Shooting) {
            action {
                towerMotor.setPercentOutput(BallConstants.towerShootingPower)
            }
        }

        state(TowerStates.ManualReverse) {
            action {
                towerMotor.setPercentOutput(BallConstants.towerManualReversePower)
            }
        }
    }

    val flyingVMachine: StateMachine<FlyingVStates> = stateMachine {
        state(FlyingVStates.Intaking) {
            action {
                flyingVMotors.setPercentOutput(BallConstants.flyingVIntakingPower)
            }
        }

        state(FlyingVStates.Shooting) {
            action {
                flyingVMotors.setPercentOutput(BallConstants.flyingVShootingPower)
            }
        }

        state(FlyingVStates.Idle) {
            tickedAction(BallConstants.flyingVIdleTimeout, {
                flyingVMotors.setPercentOutput(BallConstants.flyingVIdlePower)
                towerMachine.isInState(TowerStates.Waiting)
            }, { disable() })
        }

        state(FlyingVStates.ManualReverse) {
            action {
                flyingVMotors.setPercentOutput(BallConstants.flyingVReversingPower)
            }
        }

        disabled {
            action {
                flyingVMotors.stop()
            }
        }
    }

    val intakeMachine: StateMachine<IntakeStates> = stateMachine {
        state(IntakeStates.Stowed) {
            entry {
                resetIntakeArm()
            }

            rtAction { timestamp, dt ->
                //updateIntakeArm(0.0.Radians)
                updateIntakeArm((-90.0).Degrees.toRadians())
                intakeWheelsMotor.stop()
            }
        }

        state(IntakeStates.Intake) {
            entry {
                resetIntakeArm()
            }

            rtAction { timestamp, dt ->
                updateIntakeArm((-90.0).Degrees.toRadians())
                intakeWheelsMotor.setPercentOutput(1.0)
            }
        }

        state(IntakeStates.GoToStow) {
            timeout(0.3.Seconds, IntakeStates.Stowed)

            entry {
                resetIntakeArm()
            }

            rtAction { timestamp, dt ->
                //updateIntakeArm(0.0.Radians)
                updateIntakeArm((-90.0).Degrees.toRadians())
                intakeWheelsMotor.setPercentOutput(-0.75)
            }
        }

        disabled {
            action {
                intakeArmGearbox.stop()
                intakeWheelsMotor.stop()
            }
        }
    }

    override fun setup() {
        useHardware(towerMotor) {
            inverted = true
            pidController.p = BallConstants.intakeArmKp
        }

        useHardware(intakeArmMotor) {
            inverted = true
            idleMode = CANSparkMax.IdleMode.kBrake
            enableVoltageCompensation(12.0)
            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10)
            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10)
        }

        useHardware(flyingVMotors) {
            setNeutralMode(NeutralMode.Brake)
            inverted = true
        }

        intakeArmGearbox.setAngularPosition(0.0.Radians)

        on (Events.ENABLED) {
            towerMachine.setState(TowerStates.Waiting)
            intakeMachine.setState(IntakeStates.Stowed)
        }
    }
}