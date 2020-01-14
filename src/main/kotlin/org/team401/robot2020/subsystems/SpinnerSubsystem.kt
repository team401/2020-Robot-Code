package org.team401.robot2020.subsystems

import com.revrobotics.ColorMatch
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.component.dsl.createBrushlessSparkMax
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import org.snakeskin.event.Events
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import org.team401.robot2020.control.spinner.SpinnerAlgorithm


object SpinnerSubsystem : Subsystem() {
    private val wheelSpark = Hardware.createBrushlessSparkMax(1)

    private val pusherPiston = Solenoid(1)

    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private val colorMatcher = ColorMatch()

    private val wheelRadius = 2.0.Inches
    private val spinnerRadius = 32.0.Inches

    private val blueTarget = ColorMatch.makeColor(0.0, 0.0, 255.0)
    private val greenTarget = ColorMatch.makeColor(0.0, 255.0, 0.0)
    private val redTarget = ColorMatch.makeColor(255.0, 0.0, 0.0)
    private val yellowTarget = ColorMatch.makeColor(255.0, 255.0, 0.0)

    private val desiredColor = SpinnerColor.Yellow
    private val desiredRotations = 4.25.Revolutions

    private val accel = 100.0.RevolutionsPerMinutePerSecond.toRevolutionsPerSecondPerSecond().value
    private var power = 0.0
    private const val maxPower = 0.5


    enum class States {
        Position,
        Rotation,
        Disabled
    }

    enum class SpinnerColor {
        Red,
        Green,
        Blue,
        Yellow,
        Unknown
    }

    private val SpinnerMachine : StateMachine<States> = stateMachine() {
        state(States.Position) {
            var wheelRotation = 0.0
            var startTime = 0.0.Seconds

            entry {
                val detectedColor = colorSensor.color
                var currentColor = SpinnerColor.Unknown
                val matchResult = colorMatcher.matchClosestColor(detectedColor)

                if (matchResult.color == blueTarget) {
                    currentColor = SpinnerColor.Blue
                } else if (matchResult.color == redTarget) {
                    currentColor = SpinnerColor.Red
                } else if (matchResult.color ==  greenTarget) {
                    currentColor = SpinnerColor.Green
                } else {
                    currentColor = SpinnerColor.Yellow
                }

                val spinnerDesiredDegrees = SpinnerAlgorithm.calculateSpinnerRotation(currentColor, desiredColor)

                wheelRotation = ((spinnerRadius.value / wheelRadius.value) * spinnerDesiredDegrees.value) / 360.0.Degrees.value
                wheelSpark.setAngularPosition(0.0.Revolutions)
                startTime = 0.0.Seconds

                pusherPiston.set(false)

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