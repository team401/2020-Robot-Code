package org.team401.robot2020.util

import org.snakeskin.component.IDigitalInputChannel

class InvertedDigitalInput(val channel: IDigitalInputChannel): IDigitalInputChannel {
    override fun getState(): Boolean {
        return !channel.getState()
    }
}

fun  IDigitalInputChannel.inverted() = InvertedDigitalInput(this)