package org.team401.robot2020.subsystems

import org.snakeskin.dsl.StateMachine
import org.snakeskin.dsl.Subsystem
import org.snakeskin.dsl.stateMachine

object HangingSubsystem: Subsystem() {

    enum class States {
        Enabled,
        Disabled
    }

    val HangingMachine: StateMachine<States> = stateMachine() {

    }
}