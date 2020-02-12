package org.team401.robot2020.subsystems

import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.team401.robot2020.config.HardwareMap

object BallSubsystem : Subsystem() {
    val feederMotor = Hardware.createBrushlessSparkMax(HardwareMap.BallMap.feederId)
    val bottomGate = Hardware.createDigitalInputChannel(HardwareMap.BallMap.bottomGate)
    val topGate = Hardware.createDigitalInputChannel(HardwareMap.BallMap.topGate)

    enum class States {
        Shuttling,
        Stationary,
        Reverse,
        Starting
    }

    val BallMachine : StateMachine<States> = stateMachine {
        state(States.Shuttling) {
            action {
                if (!(bottomGate.getState() && !topGate.getState())) {
                    feederMotor.setPercentOutput(0.5)
                } else {
                    setState(States.Stationary)
                }
            }
        }

        state(States.Stationary) {
            action {
                if (!(bottomGate.getState() && !topGate.getState())) {
                    setState(States.Shuttling)
                } else {
                    feederMotor.setPercentOutput(0.0)
                }
            }
        }

        state(States.Reverse) {
            action {
                feederMotor.setPercentOutput(-0.5)
            }
        }

        state(States.Starting) {
            action {
                if (!bottomGate.getState()){
                    setState(States.Shuttling)
                } else {
                    feederMotor.setPercentOutput(0.5)
                }
            }
        }
    }

    override fun setup() {
        on (Events.TELEOP_ENABLED) {
            BallMachine.setState(States.Stationary)
            println("Top: $topGate")
            println("Bottom: $bottomGate")
        }
    }
}