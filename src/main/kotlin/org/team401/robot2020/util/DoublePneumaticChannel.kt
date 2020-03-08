package org.team401.robot2020.util

import org.snakeskin.component.IPneumaticChannel

/**
 * Simple implementation of a double pneumatic solenoid
 */
class DoublePneumaticChannel(private val forwardChannel: IPneumaticChannel, private val reverseChannel: IPneumaticChannel): IPneumaticChannel {
    override fun setState(state: Boolean) {
        forwardChannel.setState(state)
        reverseChannel.setState(!state)
    }
}