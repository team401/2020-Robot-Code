package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.logic.LowPass
import org.snakeskin.measure.*
import org.snakeskin.utility.value.AsyncDouble
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
    private val pid = ProfiledPIDController(TurretDynamics.kP, 0.0, 0.0, TrapezoidProfile.Constraints(90.0._rev_per_min.toRadiansPerSecond().value, (90.0 * 6)._rev_per_min_per_s.toRadiansPerSecondPerSecond().value), .01)

    var targetAngle by AsyncDouble(0.0)

    override fun action() {
        //println(encoder.getAngularPosition())
    }

    public fun getTurretAngle(): Rotation2d {
        return Rotation2d.fromRadians(encoder.getAngularPosition().toRadians().value)
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
                /*
                if (encoder.getAngularVelocity() >= .0001._ul) {
                    println(rotationSpark.getOutputVoltage())
                    disable()
                } else {
                    power += .00001
                    rotationSpark.setPercentOutput(power)
                }

                 */
                //rotationSpark.setPercentOutput(1.0)
                //println((rotationSpark.getOutputVoltage() - TurretDynamics.kS) / encoder.getAngularVelocity().toRadiansPerSecond().value)
            }
        }

        state(States.Spin){
            entry {
                pid.reset(encoder.getAngularPosition().toRadians().value)
            }

            rtAction { timestamp, dt ->
                val aimingParams = RobotState.getTurretAimingParameters(timestamp)
                val currentAngle = encoder.getAngularPosition().toRadians().value
                val targetAngle = Rotation2d(aimingParams.desiredAngle.radians, true).radians//RobotState.getFieldToVehicle(timestamp).rotation.inverse().radians
                val feedbackVolts = pid.calculate(currentAngle, targetAngle)
                val outputVelocity = pid.setpoint.velocity
                val ffVolts = model.calculate(outputVelocity - RobotState.vehicleVelocityPredicted.dtheta)

                val power = (feedbackVolts + ffVolts) / 12.0
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
        encoder.setAngularPosition(0.0.Revolutions)
        SmartDashboard.putNumber("turret_kP", 0.0)
    }
}