package org.team401.robot2020.subsystems

import com.revrobotics.ColorMatch
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.component.dsl.createBrushlessSparkMax
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import org.snakeskin.event.Events
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import org.team401.robot2020.control.spinner.*
import org.team401.robot2020.config.SpinnerColors.redTarget
import org.team401.robot2020.config.SpinnerColors.blueTarget
import org.team401.robot2020.config.SpinnerColors.greenTarget
import org.team401.robot2020.config.SpinnerColors.yellowTarget

object SpinnerSubsystem : Subsystem() {
    private val wheelSpark = Hardware.createBrushlessSparkMax(1)

    private val pusherPiston = Solenoid(1)

    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private val colorMatcher = ColorMatch()

    private val wheelRadius = 1.0.Inches
    private val spinnerRadius = 32.0.Inches

    private val desiredColor = SpinnerColor.Yellow
    private val desiredRotations = 4.25.Revolutions

    private val accel = 100.0.RevolutionsPerMinutePerSecond.toRevolutionsPerSecondPerSecond().value
    private var power = 0.0

    private const val maxPower = 0.1

    enum class States {
        Position,
        Rotation,
        Disabled,
        ManualControl
    }

    enum class SpinnerColor {
        Red,
        Green,
        Blue,
        Yellow,
        Unknown
    }

    val SpinnerMachine : StateMachine<States> = stateMachine() {
        state(States.Position) {
            var wheelRotation = 0.0
            var startTime = 0.0.Seconds

            entry {
                val detectedColor = colorSensor.color
                val currentColor: SpinnerColor
                val matchResult = colorMatcher.matchClosestColor(detectedColor)

                currentColor = SpinnerAlgorithms.matchColor(matchResult)

                val spinnerDesiredDegrees = SpinnerAlgorithms.calculateSpinnerRotation(currentColor, desiredColor)

                wheelRotation = ((spinnerRadius.value / wheelRadius.value) * spinnerDesiredDegrees.value) / 360.0.Degrees.value
                wheelSpark.setAngularPosition(0.0.Revolutions)
                startTime = 0.0.Seconds

                pusherPiston.set(true)

            }

            rtAction {timestamp, _ ->
                if (startTime == 0.0.Seconds) {
                    startTime = timestamp
                }

                val elapsedTime = (timestamp - startTime)

                power = elapsedTime.value * accel
                if (power >= maxPower) {
                    power = maxPower
                }

                if (wheelSpark.getAngularPosition().value >= wheelRotation) {
                    power = 0.0
                }
                wheelSpark.setPercentOutput(power)
            }
        }

        state(States.Rotation) {
            val wheelRotation = ((spinnerRadius.value / wheelRadius.value) * desiredRotations.value).Revolutions
            var startTime = 0.0.Seconds

            entry {
                startTime = 0.0.Seconds
                wheelSpark.setAngularPosition(0.0.Revolutions)

                pusherPiston.set(true)
            }

            rtAction { timestamp, _ ->
                if (startTime == 0.0.Seconds) {
                    startTime = timestamp
                }
                val elapsedTime = (timestamp - startTime)

                power = elapsedTime.value * accel
                if (power >= maxPower) {
                    power = maxPower
                }

                if (wheelSpark.getAngularPosition().value >= wheelRotation.value) {
                    power = 0.0
                }
                wheelSpark.setPercentOutput(power)
            }
        }

        state(States.ManualControl) {
            entry {
                pusherPiston.set(true)
            }

            action {
                power = maxPower
                wheelSpark.setPercentOutput(power)
            }
        }

        state(States.Disabled) {
            entry {
                pusherPiston.set(false)
            }

            action {
                wheelSpark.setPercentOutput(0.0)
            }
        }
    }

    override fun setup() {
        colorMatcher.addColorMatch(blueTarget)
        colorMatcher.addColorMatch(greenTarget)
        colorMatcher.addColorMatch(redTarget)
        colorMatcher.addColorMatch(yellowTarget)

        on(Events.TELEOP_ENABLED) {
            SpinnerMachine.setState(States.Disabled)
        }
    }
}