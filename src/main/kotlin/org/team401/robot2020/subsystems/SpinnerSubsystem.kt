package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.ColorMatch
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import org.snakeskin.component.SmartGearbox
import org.snakeskin.measure.acceleration.angular.AngularAccelerationMeasureRevolutionsPerSecondPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRevolutions
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.control.spinner.*
import org.team401.robot2020.config.constants.SpinnerConstants

object SpinnerSubsystem : Subsystem() {
    private val spinnerMotor = Hardware.createBrushlessSparkMax(CANDevices.spinnerMotor.canID)

    //This gearbox includes both the reduction of the actual gearbox, as well as the reduction between our wheel and the control panel
    private val spinnerGearbox = SmartGearbox(
        spinnerMotor,
        ratioToSensor = SpinnerConstants.effectiveRatio
    )

    //private val pusherPiston = Solenoid(1) //TODO put me back

    private val colorSensor by lazy { ColorSensorV3(I2C.Port.kOnboard) }
    private val colorMatcher = ColorMatch()

    enum class SpinnerStates {
        PositionObjective,
        RotationObjective,
        ManualControl
    }

    enum class SpinnerColor {
        Red,
        Green,
        Blue,
        Yellow,
        Unknown
    }

    val spinnerMachine: StateMachine<SpinnerStates> = stateMachine() {
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

        fun updateRotation(accel: AngularAccelerationMeasureRevolutionsPerSecondPerSecond): Boolean {
            val timestamp = readTimestamp()
            val decelDistance = rotationTarget.abs() - ((lastVelocity.value * lastVelocity.value) / (2.0 * accel.value)).Revolutions
            if ((spinnerGearbox.getAngularPosition() - startPosition).abs() >= decelDistance) {
                if (decelStartTime == 0.0.Seconds) {
                    decelStartTime = timestamp
                }
                val elapsedTime = (timestamp - decelStartTime)
                val speed = (lastVelocity - (accel * elapsedTime)).coerceAtLeast(0.0._rev_per_s)
                val power = speed / SpinnerConstants.maxFreeSpeed
                spinnerGearbox.setPercentOutput(power * rotationTarget.sign())
                if (power == 0.0) {
                    spinnerGearbox.stop()
                    return true
                }
            } else {
                val elapsedTime = (timestamp - startTime)
                val speed = (accel * elapsedTime).coerceAtMost(
                    SpinnerConstants.cruiseVelocity
                )
                val power = speed / SpinnerConstants.maxFreeSpeed
                spinnerGearbox.setPercentOutput(power * rotationTarget.sign())
                lastVelocity = speed
            }
            return false
        }

        state(SpinnerStates.PositionObjective) {
            val desiredColor = SpinnerColor.Blue

            entry {
                val detectedColor = colorSensor.color
                val currentColor: SpinnerColor
                val matchResult = colorMatcher.matchClosestColor(detectedColor)
                currentColor = SpinnerAlgorithms.matchColor(matchResult)

                val desired = SpinnerAlgorithms.calculateSpinnerRotation(currentColor, desiredColor).toRevolutions()
                beginRotation(desired)
                //pusherPiston.set(true)
                delay(SpinnerConstants.deployDelay)
            }

            action {
                if (updateRotation(SpinnerConstants.positionAccel)) disable()
            }
        }


        state(SpinnerStates.RotationObjective) {
            entry {
                beginRotation(SpinnerConstants.rotationDistance)
                //pusherPiston.set(true)
                delay(SpinnerConstants.deployDelay)
            }

            action {
                if (updateRotation(SpinnerConstants.rotationAccel)) disable()
            }
        }

        state(SpinnerStates.ManualControl) {
            entry {
                //pusherPiston.set(true)
            }

            action {
                spinnerGearbox.setPercentOutput(SpinnerConstants.cruiseVelocity / SpinnerConstants.maxFreeSpeed)
            }
        }

        disabled {
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
        colorMatcher.addColorMatch(SpinnerConstants.blueColor)
        colorMatcher.addColorMatch(SpinnerConstants.greenColor)
        colorMatcher.addColorMatch(SpinnerConstants.redColor)
        colorMatcher.addColorMatch(SpinnerConstants.yellowColor)

        useHardware(spinnerMotor) {
            idleMode = CANSparkMax.IdleMode.kBrake
            inverted = true
        }
    }
}