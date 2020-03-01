package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import org.snakeskin.component.impl.*
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.utility.Ticker
import org.team401.robot2020.config.constants.BallConstants
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.PneumaticDevices

/**
 * Ball handling subsystem, including the intake, flying V, and tower components.
 */
object BallSubsystem : Subsystem() {
    //<editor-fold desc="Hardware Devices">
    private val flyingVMotorA = Hardware.createVictorSPX(
        CANDevices.flyingVMotorA.canID,
        mockProducer = NullVictorSpxDevice.producer
    )

    private val flyingVMotorB = Hardware.createVictorSPX(
        CANDevices.flyingVMotorB.canID,
        mockProducer = NullVictorSpxDevice.producer
    )

    private val towerMotor = Hardware.createTalonFX(
        CANDevices.towerMotor.canID,
        mockProducer = NullTalonFxDevice.producer
    )

    private val intakeWheelsMotor = Hardware.createVictorSPX(
        CANDevices.intakeWheelsMotor.canID,
        mockProducer = NullVictorSpxDevice.producer
    )

    private val intakeExtenderPistons = Hardware.createPneumaticChannel(
        PneumaticDevices.intakeExtenders,
        mockProducer = NullPneumaticChannel.producer
    )


    private val bottomGateSensor = NullDigitalInputChannel.INSTANCE /*Hardware.createDigitalInputChannel(
        DIOChannels.towerBottomGateSensor,
        halMock = true
    ).inverted() */

    private val topGateSensor = NullDigitalInputChannel.INSTANCE /*Hardware.createDigitalInputChannel(
        DIOChannels.towerTopGateSensor,
        halMock = true
    ).inverted() */
    //</editor-fold>

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
            var count = 0

            val shooterTicker = Ticker({true}, .15.Seconds, actionRate)

            entry {
                count = 0
            }

            rtAction { timestamp, dt ->
                if (ShooterSubsystem.isShotReady()) {
                    count++
                } else {
                    count = 0
                }

                if (count >= 15) {
                    towerMotor.setPercentOutput(BallConstants.towerShootingPower)
                    count = 15
                } else {
                    towerMotor.stop()
                }
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
                flyingVMotorA.setPercentOutput(BallConstants.flyingVLeftIntakingPower)
                flyingVMotorB.setPercentOutput(BallConstants.flyingVRightIntakingPower)
            }
        }

        state(FlyingVStates.Shooting) {
            action {
                flyingVMotorA.setPercentOutput(BallConstants.flyingVShootingPower)
                flyingVMotorB.setPercentOutput(BallConstants.flyingVShootingPower)
            }
        }

        state(FlyingVStates.Idle) {
            tickedAction(BallConstants.flyingVIdleTimeout, {
                flyingVMotorA.setPercentOutput(BallConstants.flyingVIdlePower)
                flyingVMotorB.setPercentOutput(BallConstants.flyingVIdlePower)
                towerMachine.isInState(TowerStates.Waiting)
            }, { disable() })
        }

        state(FlyingVStates.ManualReverse) {
            action {
                flyingVMotorA.setPercentOutput(BallConstants.flyingVReversingPower)
                flyingVMotorB.setPercentOutput(BallConstants.flyingVReversingPower)
            }
        }

        disabled {
            action {
                flyingVMotorA.stop()
                flyingVMotorB.stop()
            }
        }
    }

    val intakeMachine: StateMachine<IntakeStates> = stateMachine {
        state(IntakeStates.Stowed) {
            entry {
                intakeExtenderPistons.setState(false)
            }

            action {
                intakeWheelsMotor.stop()
            }
        }

        state(IntakeStates.Intake) {
            entry {
                intakeExtenderPistons.setState(true)
            }

            action {
                intakeWheelsMotor.setPercentOutput(1.0)
            }
        }

        disabled {
            action {
                intakeExtenderPistons.setState(false)
                intakeWheelsMotor.stop()
            }
        }
    }

    override fun setup() {
        towerMotor.invert(true)
        flyingVMotorA.invert(true)
        flyingVMotorB.invert(false)

        useHardware(towerMotor) {
        }

        useHardware(flyingVMotorA) {
            setNeutralMode(NeutralMode.Brake)
        }

        useHardware(flyingVMotorB) {
            setNeutralMode(NeutralMode.Brake)
        }

        intakeExtenderPistons.setState(false)

        on (Events.ENABLED) {
            towerMachine.setState(TowerStates.Waiting)
            intakeMachine.setState(IntakeStates.Stowed)
        }
    }
}