package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure._ul
import org.team401.robot2020.config.HardwareMap
import org.team401.robot2020.config.TurretDynamics
import org.team401.robot2020.control.robot.RobotState
import org.team401.taxis.geometry.Rotation2d

object TurretSubsystem: Subsystem () {
    private val rotationSpark = Hardware.createBrushlessSparkMax(HardwareMap.TurretMap.turretRotationId, SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice)
    private val encoder = Hardware.createDIOEncoder(
        HardwareMap.TurretMap.turretEncoderA,
        HardwareMap.TurretMap.turretEncoderB,
        8192.0,
        true
    )

    private val model = SimpleMotorFeedforward(TurretDynamics.kS, TurretDynamics.kV)

    override fun action() {
        //println(encoder.getAngularPosition())
    }

    public fun getTurretAngle(): Rotation2d {
        return Rotation2d.fromDegrees(encoder.getAngularPosition().toDegrees().value)
    }


    enum class States {
        Enabled,
        Characterize,
        Spin,
        Disabled
    }

    val TurretMachine: StateMachine<States> = stateMachine {
        state(States.Enabled) {

        }

        state(States.Characterize) {
            var power = 0.0
            entry {
                power = 0.0
            }

            action {
                if (encoder.getAngularVelocity() >= .001._ul) {
                    println(rotationSpark.getOutputVoltage())
                    disable()
                } else {
                    power += .0001
                    rotationSpark.setPercentOutput(power)
                }
                //rotationSpark.setPercentOutput(1.0)
                //println(rotationSpark.getOutputVoltage() / encoder.getAngularVelocity().toRadiansPerSecond().value)
            }
        }

        state(States.Spin){
            rtAction { timestamp, dt ->
                val velocity = RobotState.vehicleVelocityPredicted.dtheta
                val power = model.calculate(-velocity) / 12.0
                rotationSpark.setPercentOutput(power)
            }
        }

        state(States.Disabled) {

        }
    }

    override fun setup() {
        useHardware(rotationSpark) {
            idleMode = CANSparkMax.IdleMode.kBrake
            enableVoltageCompensation(12.0)
    }
        on(Events.TELEOP_ENABLED) {
            TurretMachine.setState(States.Spin)
        }
    }
}