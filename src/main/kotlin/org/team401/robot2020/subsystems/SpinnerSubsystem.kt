package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.ColorMatch
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.component.dsl.createBrushlessSparkMax
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import org.snakeskin.event.Events
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.dsl.useHardware
import org.team401.robot2020.config.Geometry.DrivetrainGeometry.wheelRadius
import org.team401.robot2020.config.SpinnerParameters
import org.team401.robot2020.control.spinner.*
import org.team401.robot2020.config.SpinnerParameters.redTarget
import org.team401.robot2020.config.SpinnerParameters.blueTarget
import org.team401.robot2020.config.SpinnerParameters.greenTarget
import org.team401.robot2020.config.SpinnerParameters.yellowTarget

object SpinnerSubsystem : Subsystem() {
    private val wheelSpark = Hardware.createBrushlessSparkMax(1)

    //This gearbox includes both the reduction of the actual gearbox, as well as the reduction between our wheel and the control panel
    private val spinnerGearbox = SmartGearbox(
        wheelSpark,
        ratioToSensor = SpinnerParameters.finalReduction
    )

    //private val pusherPiston = Solenoid(1) //TODO put me back

    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)
    private val colorMatcher = ColorMatch()

    private val desiredColor = SpinnerColor.Yellow

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
        /*
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

                val power = elapsedTime.value * accel


                if (power >= maxPower) {
                    power = maxPower
                }

                if (wheelSpark.getAngularPosition().value >= wheelRotation) {
                    power = 0.0
                }
                wheelSpark.setPercentOutput(power)
            }
        }
         */

        state(States.Rotation) {
            var startTime = 0.0.Seconds
            var decelStartTime = 0.0.Seconds

            entry {
                startTime = readTimestamp()
                decelStartTime = 0.0.Seconds
                spinnerGearbox.setAngularPosition(0.0.Revolutions)
                Thread.sleep(100)
                //pusherPiston.set(true)
            }

            action {
                val timestamp = readTimestamp()

                if (spinnerGearbox.getAngularPosition() >= SpinnerParameters.decelNumRevs) {
                    //Time to decelerate
                    if (decelStartTime == 0.0.Seconds) {
                        decelStartTime = timestamp
                    }
                    val elapsedTime = (timestamp - decelStartTime)
                    val speed = (SpinnerParameters.desiredControlPanelVelocity.value - (elapsedTime.value * SpinnerParameters.desiredControlPanelAccel.value)).coerceAtLeast(0.0)
                    val power = speed / SpinnerParameters.maxRps
                    spinnerGearbox.setPercentOutput(power)
                    if (power == 0.0) {
                        spinnerGearbox.stop()
                        setState(States.Disabled)
                    }
                } else {
                    //Accelerate / cruise
                    val elapsedTime = (timestamp - startTime)
                    val speed = (elapsedTime.value * SpinnerParameters.desiredControlPanelAccel.value).coerceAtMost(
                        SpinnerParameters.desiredControlPanelVelocity.value
                    )
                    spinnerGearbox.setPercentOutput(speed / SpinnerParameters.maxRps)
                }
            }
        }

        state(States.ManualControl) {
            entry {
                //pusherPiston.set(true)
            }

            action {
                spinnerGearbox.setPercentOutput(SpinnerParameters.desiredControlPanelVelocity.value / SpinnerParameters.maxRps)
            }
        }

        state(States.Disabled) {
            entry {
                //pusherPiston.set(false)
            }

            action {
                spinnerGearbox.stop()
            }
        }
    }

    override fun setup() {
        colorMatcher.addColorMatch(blueTarget)
        colorMatcher.addColorMatch(greenTarget)
        colorMatcher.addColorMatch(redTarget)
        colorMatcher.addColorMatch(yellowTarget)

        useHardware(wheelSpark) {
            idleMode = CANSparkMax.IdleMode.kBrake
        }

        on(Events.TELEOP_ENABLED) {
            SpinnerMachine.setState(States.Disabled)
        }
    }
}