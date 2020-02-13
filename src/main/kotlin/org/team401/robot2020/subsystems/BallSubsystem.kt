package org.team401.robot2020.subsystems

import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Seconds
import org.team401.robot2020.config.HardwareMap
import org.team401.robot2020.util.inverted

object BallSubsystem : Subsystem() {
    val feederMotor = Hardware.createBrushlessSparkMax(HardwareMap.BallMap.feederId)
    val bottomGate = Hardware.createDigitalInputChannel(HardwareMap.BallMap.bottomGate).inverted()
    val topGate = Hardware.createDigitalInputChannel(HardwareMap.BallMap.topGate).inverted()

    enum class States {
        Waiting, //Default state, waiting for ball to trip the bottom gate
        Feeding, //Ball tripped gate, waiting for ball to clear bottom gate
        Spacing, //Timed state to advance ball forward to ensure proper spacing of the balls
        Reversing, //Top gate is on and bottom gate is off, reversing balls to ensure future spacing
        Full //Tower is full (both top and bottom gate are on)
    }

    val feedingPower = .2
    val spacingPower = .05
    val reversingPower = -.2

    val BallMachine : StateMachine<States> = stateMachine {
        state(States.Waiting) {
            action {
                feederMotor.stop() //No motion during waiting

                val bottomState = bottomGate.getState()
                val topState = topGate.getState()

                when {
                    (topState && bottomState) -> setState(States.Full) //Both gates triggered, tower is full
                    (topState && !bottomState) -> setState(States.Reversing) //Balls are not at the start of the tower, reverse
                    (bottomState) -> setState(States.Feeding) //Ball is ready to enter the tower
                }
            }
        }

        state(States.Feeding) {
            action {
                feederMotor.setPercentOutput(feedingPower)

                val bottomState = bottomGate.getState()
                val topState = topGate.getState()

                when {
                    (topState && bottomState) -> setState(States.Full) //Both gates triggered, tower is full
                    (topState) -> setState(States.Reversing) //Spacing is somehow off and a new ball is being entered
                    (!bottomState) -> setState(States.Spacing) //New ball has entered the system
                }
            }
        }

        state(States.Spacing) {
            timeout(.03.Seconds, States.Waiting) //Timed state to go back into waiting

            action {
                feederMotor.setPercentOutput(spacingPower)
            }
        }

        state(States.Reversing) {
            action {
                feederMotor.setPercentOutput(reversingPower)

                val bottomState = bottomGate.getState()
                val topState = topGate.getState()

                when {
                    (bottomState) -> setState(States.Spacing)
                }
            }
        }

        state(States.Full) {
            action {
                feederMotor.stop()

                val bottomState = bottomGate.getState()
                val topState = topGate.getState()

                if (!bottomState || !topState) setState(States.Waiting)
            }
        }
    }

    override fun action() {
        println("T: ${topGate.getState()}\t\t\t\t B: ${bottomGate.getState()}")
    }

    override fun setup() {
        on (Events.TELEOP_ENABLED) {
            BallMachine.setState(States.Waiting)
        }
    }
}