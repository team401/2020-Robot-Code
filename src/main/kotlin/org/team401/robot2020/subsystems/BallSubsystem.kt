package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.ArmFeedforward
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.HardwareSparkMaxDevice
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.component.impl.NullTalonSrxDevice
import org.snakeskin.component.impl.NullVictorSpxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.runtime.SnakeskinRuntime
import org.snakeskin.subsystem.States
import org.snakeskin.subsystem.SubsystemCheckContext
import org.snakeskin.utility.value.AsyncBoolean
import org.team401.robot2020.config.constants.BallConstants
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.DIOChannels
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.util.CharacterizationRoutine
import org.team401.robot2020.util.LoggerSession
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

    private val intakePivotMotor = Hardware.createBrushlessSparkMaxWithEncoder(
        CANDevices.intakePivotMotor.canID,
        8192, //REV Through Bore Encoder
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val intakePivotGearbox = SmartGearbox(intakePivotMotor)

    private val bottomGateSensor = Hardware.createDigitalInputChannel(
        DIOChannels.towerBottomGateSensor,
        halMock = true
    ).inverted()

    private val topGateSensor = Hardware.createDigitalInputChannel(
        DIOChannels.towerTopGateSensor,
        halMock = true
    ).inverted()
    //</editor-fold>

    enum class TowerStates {
        Waiting, //Default state, waiting for ball to trip the bottom gate
        Feeding, //Ball tripped gate, waiting for ball to clear bottom gate
        Spacing, //Timed state to advance ball forward to ensure proper spacing of the balls
        Reversing, //Top gate is on and bottom gate is off, reversing balls to ensure future spacing
        Full, //Tower is full (both top and bottom gate are on)
        Shooting,
        ManualReverse
    }

    enum class FlyingVStates {
        Feed,
        Reverse
    }

    val flyingVMachine: StateMachine<FlyingVStates> = stateMachine {
        state(FlyingVStates.Feed) {
            action {
                //flyingVMotors.setPercentOutput(BallConstants.flyingVFeedPower)
            }
        }

        state(FlyingVStates.Reverse) {
            action {
                //flyingVMotors.setPercentOutput(BallConstants.flyingVReversePower)
            }
        }

        disabled {
            action {
                //flyingVMotors.stop()
            }
        }
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
                towerMotor.setAngularVelocitySetpoint(BallConstants.towerFeedingRate.toRevolutionsPerSecond())

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
                towerMotor.setPercentOutput(0.75)
            }
        }

        state(TowerStates.Reversing) {
            action {
                towerMotor.setPercentOutput(-0.5)
            }
        }
    }

    val armCharacterizationRoutine = CharacterizationRoutine(intakePivotGearbox, intakePivotGearbox)

    private val armModel = ArmFeedforward(
        BallConstants.intakeArmKs,
        BallConstants.intakeArmKcos,
        BallConstants.intakeArmKv
    )

    private val armController = ProfiledPIDController(6.0, 0.0, 0.0,
        TrapezoidProfile.Constraints(
            (180.0.Degrees / .5.Seconds).toRadiansPerSecond().value,
            (90.0.Degrees / .5.Seconds / .1.Seconds).toRadiansPerSecondPerSecond().value),
        RobotConstants.rtPeriod.value
    )

    enum class IntakeStates {
        Stow,
        Intake
    }

    val intakeMachine: StateMachine<IntakeStates> = stateMachine {
        state(IntakeStates.Stow) {
            entry {
                armController.reset(intakePivotGearbox.getAngularPosition().toRadians().value)
            }

            rtAction { timestamp, dt ->
                val position = intakePivotGearbox.getAngularPosition().toRadians().value
                val feedbackVolts = armController.calculate(position, (0.0).Degrees.toRadians().value)
                val ffVolts = armModel.calculate(armController.setpoint.position, armController.setpoint.velocity)

                val out = (feedbackVolts + ffVolts) / 12.0

                intakePivotGearbox.setPercentOutput(out)
                intakeWheelsMotor.stop()
            }
        }

        state(IntakeStates.Intake) {
            entry {
                armController.reset(intakePivotGearbox.getAngularPosition().toRadians().value)
            }

            rtAction { timestamp, dt ->
                val position = intakePivotGearbox.getAngularPosition().toRadians().value
                val feedbackVolts = armController.calculate(position, (-90.0).Degrees.toRadians().value)
                val ffVolts = armModel.calculate(armController.setpoint.position, armController.setpoint.velocity)

                val out = (feedbackVolts + ffVolts) / 12.0

                intakePivotGearbox.setPercentOutput(out)
                intakeWheelsMotor.setPercentOutput(1.0)
            }
        }

        disabled {
            action {
                intakePivotGearbox.stop()
                intakeWheelsMotor.stop()
            }
        }
    }

    override fun setup() {
        intakePivotGearbox.setAngularPosition(0.0.Revolutions)

        useHardware(towerMotor) {
            inverted = true
            pidController.p = 0.00015
        }

        useHardware(intakePivotMotor) {
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

        on (Events.TELEOP_ENABLED) {
            intakeMachine.setState(IntakeStates.Stow)
            towerMachine.setState(TowerStates.Waiting)
        }

        SmartDashboard.putNumber("arm_p", 0.0)
        SmartDashboard.putNumber("arm_d", 0.0)
        SmartDashboard.putNumber("tower_p", 0.0)
    }
}