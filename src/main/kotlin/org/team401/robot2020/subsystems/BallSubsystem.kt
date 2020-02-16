package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.component.impl.NullVictorSpxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.subsystem.States
import org.team401.robot2020.config.constants.BallConstants
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.DIOChannels
import org.team401.robot2020.util.inverted

/**
 * Ball handling subsystem, including the intake, flying V, and tower components.
 */
object BallSubsystem : Subsystem() {
    //<editor-fold desc="Hardware Devices">
    private val flyingVMotors = Hardware.createVictorSPX(
        CANDevices.flyingVMotors.canID,
        mockProducer = NullVictorSpxDevice.producer
    )
    private val towerMotor = Hardware.createBrushlessSparkMax(
        CANDevices.towerMotor.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )
    /*private val intakeWheelsMotor = Hardware.createBrushlessSparkMax(
        CANDevices.intakeWheelsMotor.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )*/
    private val intakeWheelsMotor = Hardware.createBrushlessSparkMax(25)
    private val intakePivotMotor = Hardware.createBrushlessSparkMaxWithEncoder(
        CANDevices.intakePivotMotor.canID,
        8192, //REV Through Bore Encoder
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val bottomGateSensor = Hardware.createDigitalInputChannel(
        DIOChannels.towerBottomGateSensor,
        halMock = true
    ).inverted()

    private val topGateSensor = Hardware.createDigitalInputChannel(
        DIOChannels.towerTopGateSensor,
        halMock = true
    ).inverted()
    //</editor-fold>
/*
    enum class TowerStates {
        Waiting, //Default state, waiting for ball to trip the bottom gate
        Feeding, //Ball tripped gate, waiting for ball to clear bottom gate
        Spacing, //Timed state to advance ball forward to ensure proper spacing of the balls
        Reversing, //Top gate is on and bottom gate is off, reversing balls to ensure future spacing
        Full //Tower is full (both top and bottom gate are on)
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
            action {
                towerMotor.setPercentOutput(BallConstants.towerFeedingPower)

                val bottomState = bottomGateSensor.getState()
                val topState = topGateSensor.getState()

                when {
                    (topState && bottomState) -> setState(TowerStates.Full) //Both gates triggered, tower is full
                    (topState) -> setState(TowerStates.Reversing) //Spacing is somehow off and a new ball is being entered
                    (!bottomState) -> setState(TowerStates.Spacing) //New ball has entered the system
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
            action {
                towerMotor.setPercentOutput(BallConstants.towerReversingPower)

                val bottomState = bottomGateSensor.getState()

                when {
                    (bottomState) -> setState(TowerStates.Spacing)
                }
            }
        }

        state(TowerStates.Full) {
            action {
                towerMotor.stop()

                val bottomState = bottomGateSensor.getState()
                val topState = topGateSensor.getState()

                if (!bottomState || !topState) setState(TowerStates.Waiting)
            }
        }
    }*/

    enum class IntakeStates {
        Intaking,
        Waiting
    }

    val intakingMachine : StateMachine<IntakeStates> = stateMachine {
        state(IntakeStates.Intaking) {
           action {
               intakeWheelsMotor.setPercentOutput(.25)
               println("intaking")

           }
        }

        state(IntakeStates.Waiting) {
            action {
                intakeWheelsMotor.setPercentOutput(0.0)
                println("waiting")
            }
        }
    }

    override fun setup() {
        useHardware(intakeWheelsMotor) {
            idleMode = CANSparkMax.IdleMode.kBrake
        }

        on (Events.TELEOP_ENABLED) {
            intakingMachine.setState(IntakeStates.Waiting)
        }
    }
}