package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.Gearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.logic.LowPass
import org.snakeskin.measure.*
import org.snakeskin.utility.value.AsyncDouble
import org.team401.robot2020.config.HardwareMap
import org.team401.robot2020.config.TurretDynamics
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.robot.TurretLimelight
import org.team401.robot2020.util.LoggerSession
import org.team401.taxis.geometry.Rotation2d

object TurretSubsystem: Subsystem () {
    private val rotationSpark = Hardware.createBrushlessSparkMax(HardwareMap.TurretMap.turretRotationId, SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice)
    private val encoder = Hardware.createDIOEncoder(
        HardwareMap.TurretMap.turretEncoderA,
        HardwareMap.TurretMap.turretEncoderB,
        8192.0,
        false
    )

    private val gearbox = Gearbox(encoder, rotationSpark, ratioToSensor = 3.95)

    private val model = SimpleMotorFeedforward(TurretDynamics.kS, TurretDynamics.kV)
    private val pid = ProfiledPIDController(TurretDynamics.kP, 0.0, 0.0, TrapezoidProfile.Constraints(70.0._rev_per_min.toRadiansPerSecond().value, (70.0 * 4)._rev_per_min_per_s.toRadiansPerSecondPerSecond().value), .01)

    var targetAngle by AsyncDouble(0.0)

    override fun action() {
        //println(gearbox.getAngularPosition())
    }

    public fun getTurretAngle(): Rotation2d {
        return Rotation2d.fromRadians(gearbox.getAngularPosition().toRadians().value)
    }


    enum class States {
        Enabled,
        Characterize,
        NoVision,
        Spin,
        Disabled
    }

    val TurretMachine: StateMachine<States> = stateMachine {
        state(States.Enabled) {
            action {
                gearbox.setPercentOutput(.25)
            }
        }

        state(States.Characterize) {
            var power = 0.0
            entry {
                power = 0.0
                pid.p = SmartDashboard.getNumber("turret_kP", 0.0)
                pid.reset(gearbox.getAngularPosition().toRadians().value)
            }

            rtAction { timestamp, dt ->
                val currentAngle = gearbox.getAngularPosition().toRadians().value
                val targetAngle = Rotation2d.fromDegrees(90.0).radians
                val feedbackVolts = pid.calculate(currentAngle, targetAngle)
                val outputVelocity = pid.setpoint.velocity
                val ffVolts = model.calculate(outputVelocity)

                val power = (feedbackVolts + ffVolts) / 12.0
                gearbox.setPercentOutput(power)
            }
                /*
                if (gearbox.getAngularVelocity() >= .01._ul) {
                    println(gearbox.getOutputVoltage())
                    disable()
                } else {
                    power += .00001
                    gearbox.setPercentOutput(power)
                }

                 */
                //gearbox.setPercentOutput(.5)
                //println((gearbox.getOutputVoltage() - TurretDynamics.kS) / gearbox.getAngularVelocity().toRadiansPerSecond().value)
        }

        state (States.NoVision) {
            entry {
                pid.reset(gearbox.getAngularPosition().toRadians().value)
            }

            rtAction { timestamp, dt ->
                val currentAngle = gearbox.getAngularPosition().toRadians().value
                val targetAngle = RobotState.getFieldToVehicle(timestamp).rotation.inverse().radians
                val feedbackVolts = pid.calculate(currentAngle, targetAngle)
                val outputVelocity = pid.setpoint.velocity
                val ffVolts = model.calculate(outputVelocity - RobotState.vehicleVelocityPredicted.dtheta)

                val power = (feedbackVolts + ffVolts) / 12.0
                gearbox.setPercentOutput(power)
            }
        }

        state(States.Spin){
            lateinit var session: LoggerSession

            entry {
                pid.reset(gearbox.getAngularPosition().toRadians().value)
                session = LoggerSession("10.4.1.162", 5801)
            }

            rtAction { timestamp, dt ->
                val aimingParams = RobotState.getTurretAimingParameters(timestamp)
                session["Timestamp (s)"] = timestamp.value
                //println(aimingParams.desiredAngle)
                val currentAngle = gearbox.getAngularPosition().toRadians().value
                val targetAngle = Rotation2d(aimingParams.desiredAngle.radians, true).radians//RobotState.getFieldToVehicle(timestamp).rotation.inverse().radians
                val feedbackVolts = pid.calculate(currentAngle, targetAngle)
                val outputVelocity = pid.setpoint.velocity
                val ffVolts = model.calculate(outputVelocity - RobotState.vehicleVelocityPredicted.dtheta)

                val power = (feedbackVolts + ffVolts) / 12.0
                gearbox.setPercentOutput(power)

                session["Desired Angle (rad)"] = pid.setpoint.position
                session["Actual Angle (rad)"] = currentAngle

                session.publish()
            }

            exit {
                session.end()
            }
        }

        state(States.Disabled) {
            action {
                gearbox.stop()
            }
        }
    }

    override fun setup() {
        useHardware(rotationSpark) {
            idleMode = CANSparkMax.IdleMode.kBrake
            enableVoltageCompensation(12.0)
            inverted = true
        }
        on(Events.TELEOP_ENABLED) {
            TurretMachine.setState(States.NoVision)
        }
        encoder.setAngularPosition(0.0.Revolutions)
        SmartDashboard.putNumber("turret_kP", 0.0)
    }
}

