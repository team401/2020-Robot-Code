package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration
import org.snakeskin.component.impl.*
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.team401.robot2020.config.constants.BallConstants
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.DIOChannels
import org.team401.robot2020.config.PneumaticChannels
import org.team401.robot2020.util.DoublePneumaticChannel
import org.team401.robot2020.util.inverted

/**
 * Ball handling subsystem, including the intake, flying V, and tower components.
 */
object BallSubsystem : Subsystem() {
    //<editor-fold desc="Hardware Devices">
    private val flyingVMotorLeft = Hardware.createVictorSPX(
        CANDevices.flyingVMotorLeft.canID,
        mockProducer = NullVictorSpxDevice.producer
    )

    private val flyingVMotorRight = Hardware.createVictorSPX(
        CANDevices.flyingVMotorRight.canID,
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

    private val intakeDeployPiston = DoublePneumaticChannel(
        Hardware.createPneumaticChannel(
            PneumaticChannels.intakeDeploySolenoidForward,
            halMock = true
        ),
        Hardware.createPneumaticChannel(
            PneumaticChannels.intakeDeploySolenoidReverse,
            halMock = true
        )
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
        Intake
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
            action(0.01.Seconds) {
                towerMotor.setPercentOutput(BallConstants.towerFeedingPower)

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

        state(TowerStates.Shooting) {
            var count = 0

            entry {
                count = 0
            }

            rtAction { timestamp, dt ->
                if (ShooterSubsystem.isShotReady()) {
                    count++
                } else {
                    count = 0
                }

                if (count >= 5) {
                    towerMotor.setPercentOutput(BallConstants.towerShootingPower)
                    count = 5
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

        disabled {
            entry {
                towerMotor.stop()
            }
        }
    }

    val flyingVMachine: StateMachine<FlyingVStates> = stateMachine {
        state(FlyingVStates.Intaking) {
            action {
                flyingVMotorLeft.setPercentOutput(BallConstants.flyingVLeftIntakingPower)
                flyingVMotorRight.setPercentOutput(BallConstants.flyingVRightIntakingPower)
            }
        }

        state(FlyingVStates.Shooting) {
            action {
                flyingVMotorLeft.setPercentOutput(BallConstants.flyingVLeftIntakingPower)
                flyingVMotorRight.setPercentOutput(BallConstants.flyingVRightIntakingPower)
            }
        }

        state(FlyingVStates.Idle) {
            tickedAction(BallConstants.flyingVIdleTimeout, {
                flyingVMotorLeft.setPercentOutput(BallConstants.flyingVLeftIntakingPower)
                flyingVMotorRight.setPercentOutput(BallConstants.flyingVRightIntakingPower)
                towerMachine.isInState(TowerStates.Waiting)
            }, { disable() })
        }

        state(FlyingVStates.ManualReverse) {
            action {
                flyingVMotorLeft.setPercentOutput(BallConstants.flyingVReversingPower)
                flyingVMotorRight.setPercentOutput(BallConstants.flyingVReversingPower)
            }
        }

        disabled {
            entry {
                flyingVMotorLeft.stop()
                flyingVMotorRight.stop()
            }
        }
    }

    val intakeMachine: StateMachine<IntakeStates> = stateMachine {
        state(IntakeStates.Stowed) {
            entry {
                intakeDeployPiston.setState(false)
            }

            action {
                intakeWheelsMotor.stop()
            }
        }

        state(IntakeStates.Intake) {
            entry {
                intakeDeployPiston.setState(true)
            }

            action {
                intakeWheelsMotor.setPercentOutput(1.0)
            }
        }

        disabled {
            entry {
                intakeDeployPiston.setState(false)
                intakeWheelsMotor.stop()
            }
        }
    }

    override fun setup() {
        towerMotor.invert(true)
        flyingVMotorLeft.invert(true)
        flyingVMotorRight.invert(false)
        towerMotor.invert(false)

        useHardware(flyingVMotorLeft) {
            setNeutralMode(NeutralMode.Brake)
        }

        useHardware(flyingVMotorRight) {
            setNeutralMode(NeutralMode.Brake)
        }

        useHardware(towerMotor) {
            setNeutralMode(NeutralMode.Brake)
            configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20.0, 0.0, 0.0), 1000)
        }

        on (Events.TELEOP_ENABLED) {
            towerMachine.setState(TowerStates.Waiting)
            intakeMachine.setState(IntakeStates.Stowed)
        }
    }
}