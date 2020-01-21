package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import org.snakeskin.dsl.*
import org.snakeskin.event.Events

object HangingSubsystem: Subsystem() {
private val spark = Hardware.createBrushlessSparkMax(1)

    enum class States {
        Forward,
        Reversed,
        Disabled
    }

    val HangingMachine: StateMachine<States> = stateMachine() {
        state(States.Forward) {
            action {
                spark.setPercentOutput(1.0)
            }
        }

        state(States.Reversed) {
            action {
                spark.setPercentOutput(-0.1)
            }
        }

        state(States.Disabled) {
            action {
                spark.setPercentOutput(0.0)
            }
        }
    }

    override fun setup() {
        useHardware(spark) {
            inverted = true
            idleMode = CANSparkMax.IdleMode.kBrake
        }

        on(Events.TELEOP_ENABLED) {
            HangingMachine.setState(States.Disabled)
        }
    }
}