package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.ColorMatch
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import org.snakeskin.event.Events
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import org.snakeskin.component.SmartGearbox
import org.snakeskin.measure.acceleration.angular.AngularAccelerationMeasureRevolutionsPerSecondPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRevolutions
import org.team401.robot2020.config.SpinnerParameters
import org.team401.robot2020.control.spinner.*
import org.team401.robot2020.config.SpinnerParameters.redTarget
import org.team401.robot2020.config.SpinnerParameters.blueTarget
import org.team401.robot2020.config.SpinnerParameters.greenTarget
import org.team401.robot2020.config.SpinnerParameters.yellowTarget
import kotlin.math.sign

object SpinnerSubsystem : Subsystem() {
    private val wheelSpark = Hardware.createBrushlessSparkMax(1)

    //This gearbox includes both the reduction of the actual gearbox, as well as the reduction between our wheel and the control panel
    private val spinnerGearbox = SmartGearbox(
        wheelSpark,
        ratioToSensor = SpinnerParameters.finalReduction
    )

    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)

    private val colorMatcher = ColorMatch()

    //private val pusherPiston = Solenoid(1) //TODO put me back

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

    val SpinnerMachine: StateMachine<States> = stateMachine() {
        var rotationTarget = 0.0.Revolutions
        var startTime = 0.0.Seconds
        var startPosition = 0.0.Revolutions
        var lastVelocity = 0.0._rev_per_s
        var decelStartTime = 0.0.Seconds

        fun beginRotation(target: AngularDistanceMeasureRevolutions) {
            rotationTarget = target
            startTime = readTimestamp()
            startPosition = spinnerGearbox.getAngularPosition()
            lastVelocity = 0.0._rev_per_s
            decelStartTime = 0.0.Seconds
        }

        fun updateRotation(accel: AngularAccelerationMeasureRevolutionsPerSecondPerSecond) {
            val timestamp = readTimestamp()
            val decelDistance = rotationTarget.abs() - ((lastVelocity.value * lastVelocity.value) / (2.0 * accel.value)).Revolutions
            if ((spinnerGearbox.getAngularPosition() - startPosition).abs() >= decelDistance) {
                if (decelStartTime == 0.0.Seconds) {
                    decelStartTime = timestamp
                }
                val elapsedTime = (timestamp - decelStartTime)
                val speed = (lastVelocity - (accel * elapsedTime)).coerceAtLeast(0.0._rev_per_s)
                val power = speed.value / SpinnerParameters.maxRps
                spinnerGearbox.setPercentOutput(power * sign(rotationTarget.value))
                if (power == 0.0) {
                    spinnerGearbox.stop()
                    disable()
                }
            } else {
                val elapsedTime = (timestamp - startTime)
                val speed = (accel * elapsedTime).coerceAtMost(
                    SpinnerParameters.desiredControlPanelVelocity
                )
                val power = speed.value / SpinnerParameters.maxRps
                spinnerGearbox.setPercentOutput(power * sign(rotationTarget.value))
                lastVelocity = speed
            }
        }

        state(States.Position) {
            val desiredColor = SpinnerColor.Blue

            entry {
                val detectedColor = colorSensor.color
                val currentColor: SpinnerColor
                val matchResult = colorMatcher.matchClosestColor(detectedColor)
                currentColor = SpinnerAlgorithms.matchColor(matchResult)

                val desired = SpinnerAlgorithms.calculateSpinnerRotation(currentColor, desiredColor).toRevolutions()
                beginRotation(desired)
                //pusherPiston.set(true)
            }

            action {
                updateRotation(SpinnerParameters.desiredControlPanelPositionAccel)
            }
        }


        state(States.Rotation) {
            entry {
                beginRotation(SpinnerParameters.desiredNumControlPanelRevs)
                //pusherPiston.set(true)
            }

            action {
                updateRotation(SpinnerParameters.desiredControlPanelRotationAccel)
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

    override fun action() {
        //val color = colorSensor.color
        //println("r: ${color.red}  g: ${color.green}  b: ${color.blue}")
    }

    override fun setup() {
        colorMatcher.addColorMatch(blueTarget)
        colorMatcher.addColorMatch(greenTarget)
        colorMatcher.addColorMatch(redTarget)
        colorMatcher.addColorMatch(yellowTarget)

        useHardware(wheelSpark) {
            idleMode = CANSparkMax.IdleMode.kBrake
            setCANTimeout(1000)
            inverted = true
        }

        on(Events.TELEOP_ENABLED) {
            SpinnerMachine.setState(States.Disabled)
        }
    }
}