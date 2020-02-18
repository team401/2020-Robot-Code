package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure._rev_per_min
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.control.VelocityProfiler
import org.team401.robot2020.util.LoggerSession

object ShooterSubsystem: Subsystem() {
    private val shooterMotorA = Hardware.createBrushlessSparkMax(
        CANDevices.shooterMotorA.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val shooterMotorB = Hardware.createBrushlessSparkMax(
        CANDevices.shooterMotorB.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val shooterGearbox = SmartGearbox(shooterMotorA, shooterMotorB, ratioToSensor = 18.0 / 30.0)

    private val kickerMotor = Hardware.createBrushlessSparkMax(
        CANDevices.kickerMotor.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    enum class FlywheelStates {
        Shoot
    }

    private val flywheelProfiler = VelocityProfiler(ShooterConstants.flywheelRampAcceleration)
    private val flywheelModel = SimpleMotorFeedforward(ShooterConstants.flywheelKs, ShooterConstants.flywheelKv)

    val flywheelMachine: StateMachine<FlywheelStates> = stateMachine {
        lateinit var session: LoggerSession

        state(FlywheelStates.Shoot) {
            entry {
                flywheelProfiler.reset()
                session = LoggerSession("10.4.1.162", 5801)
            }

            rtAction { timestamp, dt ->
                val velocityCommand = flywheelProfiler.calculate(dt, 7350.0._rev_per_min.toRevolutionsPerSecond())
                val ffVolts = flywheelModel.calculate(velocityCommand.value)

                val actualVelocity = shooterGearbox.getAngularVelocity()

                shooterGearbox.setAngularVelocitySetpoint(velocityCommand, ffVolts)

                session["Timestamp (s)"] = timestamp.value
                session["Commanded Velocity (RPM)"] = velocityCommand.toRevolutionsPerMinute().value
                session["Actual Velocity (RPM)"] = actualVelocity.toRevolutionsPerMinute().value
                session.publish()

            }

            exit {
                session.end()
            }
        }
    }

    override fun action() {
        useHardware(shooterMotorA) {
            inverted = true

            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 5)

            pidController.p = ShooterConstants.flywheelKp
            pidController.d = ShooterConstants.flywheelKd
        }

        useHardware(shooterMotorB) {
            inverted = false
        }

        on (Events.TELEOP_ENABLED) {
            flywheelMachine.setState(FlywheelStates.Shoot)
        }
    }
}