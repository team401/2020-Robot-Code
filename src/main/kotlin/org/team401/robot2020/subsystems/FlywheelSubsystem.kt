package org.team401.robot2020.subsystems

import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.dsl.createBrushlessSparkMax
import org.snakeskin.component.dsl.useHardware
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*

object FlywheelSubsystem : Subsystem() {
    private val leftSpark = Hardware.createBrushlessSparkMax(1)
    private val rightSpark = Hardware.createBrushlessSparkMax(2)

    private val gearbox = SmartGearbox(leftSpark, rightSpark)

    private val targetVelocity = 3750.0.RevolutionsPerMinute
    private val ks = 0.23311
    private val kv = 0.14613
    private val accel = 2000.0.RevolutionsPerMinutePerSecond.toRevolutionsPerSecondPerSecond().value

    enum class States {
        Enabled,
        MeasureKs,
        MeasureKv
    }

    private val FlywheelMachine: StateMachine<States> = stateMachine {
        state(States.Enabled) {
            var startTime = 0.0.Seconds
            entry {
                startTime = 0.0.Seconds
                useHardware(leftSpark) {
                    pidController.p = .0015
                }
            }

            rtAction { timestamp, _ ->
                if (startTime == 0.0.Seconds) {
                    startTime = timestamp
                }

                val elapsedTime = (timestamp - startTime).value
                val velocity = elapsedTime * accel

                if (velocity > targetVelocity.toRevolutionsPerSecond().value) {
                    gearbox.setAngularVelocitySetpoint(targetVelocity.toRevolutionsPerSecond(), ks + (kv * targetVelocity.toRevolutionsPerSecond().value))

                } else {
                    gearbox.setAngularVelocitySetpoint(velocity._rev_per_s, ks + (kv * velocity))

                }

                println(gearbox.getAngularVelocity().value - targetVelocity.toRevolutionsPerSecond().value)

            }
        }

        state(States.MeasureKs) {
            var power = 0.0

            entry {
                power = 0.0
            }

            rtAction { timestamp, _ ->
                gearbox.setPercentOutput(power)
                if (gearbox.getAngularVelocity() >= 0.001.RevolutionsPerSecond) {
                    println(gearbox.getOutputVoltage())
                } else {
                    power += 0.00001
                }
            }
        }
        state(States.MeasureKv) {
            rtAction { timestamp, _ ->
                gearbox.setPercentOutput(1.0)
                println((gearbox.getOutputVoltage() - ks) / gearbox.getAngularVelocity().value)
            }
        }
    }

    override fun setup() {
        useHardware(leftSpark) {
            inverted = true
        }

        useHardware(rightSpark) {
            inverted = false
        }

        on(Events.TELEOP_ENABLED) {
            FlywheelMachine.setState(States.Enabled)
        }
    }
}