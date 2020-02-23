package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.Gearbox
import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.HardwareSparkMaxDevice
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRevolutionsPerSecond
import org.snakeskin.utility.value.AsyncValue
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.DIOChannels
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.control.VelocityProfiler
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.util.CharacterizationRoutine
import org.team401.robot2020.util.LoggerSession
import org.team401.robot2020.util.getAcceleration
import org.team401.taxis.geometry.Rotation2d

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

    private val shooterGearbox = SmartGearbox(shooterMotorA, shooterMotorB, ratioToSensor = ShooterConstants.flywheelRatio)

    private val kickerMotor = Hardware.createBrushlessSparkMax(
        CANDevices.kickerMotor.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val turretRotationMotor = Hardware.createBrushlessSparkMax(
        CANDevices.turretRotationMotor.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val turretEncoder = Hardware.createDIOEncoder(
        DIOChannels.turretRotationEncoderA,
        DIOChannels.turretRotationEncoderB,
        8192.0,
        true,
        halMock = true
    )

    private val turretRotationGearbox = Gearbox(turretEncoder, turretRotationMotor, ratioToSensor = ShooterConstants.turretRatio)

    private val flywheelProfiler = VelocityProfiler(ShooterConstants.flywheelRampAcceleration)
    private val flywheelModel = SimpleMotorFeedforward(
        ShooterConstants.flywheelKs,
        ShooterConstants.flywheelKv,
        ShooterConstants.flywheelKa
    )

    private val flywheelController = PIDController(ShooterConstants.flywheelKp, 0.0, 0.0)

    private val turretConstraints = TrapezoidProfile.Constraints(
        ShooterConstants.turretVelocity.value,
        ShooterConstants.turretAccel.value
    )
    private val turretController by lazy { ProfiledPIDController(
        ShooterConstants.turretKp, 0.0,
        ShooterConstants.turretKd,
        turretConstraints,
        0.01
    ) }
    private val turretModel = SimpleMotorFeedforward(
        ShooterConstants.turretKs,
        ShooterConstants.turretKv,
        ShooterConstants.turretKa
    )

    private var lastTurretSetpoint = TrapezoidProfile.State()

    val turretCharacterizationRoutine = CharacterizationRoutine(turretRotationGearbox, turretRotationGearbox)
    val shooterCharacterizationRoutine = CharacterizationRoutine(shooterGearbox, shooterGearbox)

    fun getTurretAngle(): Rotation2d {
        return Rotation2d.fromRadians(turretRotationGearbox.getAngularPosition().toRadians().value)
    }

    enum class TurretStates {
        Test,
        Homing, //Turret is homing
        Hold, //Turret is holding its current position
        LockToZero, //Turret is pointed straight ahead
        CoordinatedControl //Turret is following instructions from the RobotState
    }

    enum class FlywheelStates {
        Test,
        PreSpin, //Flywheel is spun up to the initial velocity
        CoordinatedControl, //Flywheel is following instructions from the RobotState
        Manual //Flywheel has been taken control of manually and is following user input
    }

    enum class KickerStates {
        Kick
    }

    private var flywheelManualSpeed = 0.0.RadiansPerSecond
    @Synchronized get

    /**
     * Bumps up the speed of the flywheel.  Only has an effect in manual mode.
     */
    @Synchronized fun flywheelBumpUp() {
        flywheelManualSpeed += ShooterConstants.flywheelBump
        if (flywheelManualSpeed > ShooterConstants.flywheelMaxVelocity) {
            flywheelManualSpeed = ShooterConstants.flywheelMaxVelocity
        }
    }

    /**
     * Bumps down the speed of the flywheel.  Only has an effect in manual mode.
     */
    @Synchronized fun flywheelBumpDown() {
        flywheelManualSpeed -= ShooterConstants.flywheelBump
        if (flywheelManualSpeed < 0.0.RadiansPerSecond) {
            flywheelManualSpeed = 0.0.RadiansPerSecond
        }
    }

    private fun updateFlywheel(dt: TimeMeasureSeconds, velocity: AngularVelocityMeasureRadiansPerSecond) {
        flywheelProfiler.calculate(dt, velocity)
        val feedbackVolts = flywheelController.calculate(shooterGearbox.getAngularVelocity().toRadiansPerSecond().value, flywheelProfiler.velocityCommand.value)
        val ffVolts = flywheelModel.calculate(
            flywheelProfiler.velocityCommand.value,
            flywheelProfiler.accelerationCommand.value
        )

        shooterGearbox.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
    }

    private fun resetTurret() {
        turretController.reset(turretRotationGearbox.getAngularPosition().toRadians().value)
        lastTurretSetpoint = TrapezoidProfile.State()
    }

    private fun updateTurret(target: AngularDistanceMeasureRadians) {
        val feedbackVolts = turretController.calculate(turretRotationGearbox.getAngularPosition().toRadians().value, target.value)
        val ffVolts = turretModel.calculate(
            turretController.setpoint.velocity,
            turretController.getAcceleration(turretConstraints, lastTurretSetpoint)
        )
        lastTurretSetpoint = turretController.setpoint

        turretRotationGearbox.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
    }

    val flywheelMachine: StateMachine<FlywheelStates> = stateMachine {
        state(FlywheelStates.Test) {
        }

        state(FlywheelStates.PreSpin) {
            entry {
                flywheelProfiler.reset(shooterGearbox.getAngularVelocity().toRadiansPerSecond())
            }

            rtAction { timestamp, dt ->
                updateFlywheel(dt, ShooterConstants.flywheelDefaultSpeed)
                println(shooterGearbox.getAngularVelocity().toRevolutionsPerMinute().value)
            }
        }

        state(FlywheelStates.CoordinatedControl) {
            entry {
                flywheelProfiler.reset(shooterGearbox.getAngularVelocity().toRadiansPerSecond())
            }

            rtAction { timestamp, dt ->
                val aimingParameters = RobotState.getTurretAimingParameters(timestamp)
                //TODO add velocity calculation (LUT/regression?)
                updateFlywheel(dt, 0.0.RadiansPerSecond)
            }
        }

        state(FlywheelStates.Manual) {
            entry {
                val currentVelocity = shooterGearbox.getAngularVelocity().toRadiansPerSecond()
                flywheelProfiler.reset(currentVelocity)
                flywheelManualSpeed = currentVelocity
            }

            rtAction { timestamp, dt ->
                updateFlywheel(dt, flywheelManualSpeed)
            }
        }

        disabled {
            action {
                shooterGearbox.stop()
            }
        }
    }

    val kickerMachine: StateMachine<KickerStates> = stateMachine {
        state(KickerStates.Kick) {
            action {
                kickerMotor.setPercentOutput(0.75)
            }
        }

        disabled {
            action {
                kickerMotor.stop()
            }
        }
    }

    var turretSetpoint by AsyncValue(0.0.Radians)

    val turretMachine: StateMachine<TurretStates> = stateMachine {
        state(TurretStates.Test) {
            entry {
                resetTurret()
            }

            rtAction { timestamp, dt ->
                updateTurret(turretSetpoint)
            }
        }

        state(TurretStates.Homing) {

        }

        state(TurretStates.Hold) {

        }

        state(TurretStates.LockToZero) {
            entry {
                resetTurret()
            }

            rtAction { timestamp, dt ->
                updateTurret(0.0.Radians)
            }
        }

        state(TurretStates.CoordinatedControl) {
            entry {
                resetTurret()
            }

            rtAction { timestamp, dt ->
                val aimingParams = RobotState.getTurretAimingParameters(timestamp)
                //val currentAngle = turretRotationGearbox.getAngularPosition().toRadians().value
                val targetAngle = Rotation2d(aimingParams.desiredAngle.radians, true).radians//RobotState.getFieldToVehicle(timestamp).rotation.inverse().radians

                updateTurret(targetAngle.Radians)
            }
        }

        disabled {
            action {
                shooterGearbox.stop()
            }
        }
    }


    override fun setup() {
        useHardware(shooterMotorA) {
            inverted = false
            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 5)
        }

        useHardware(shooterMotorB) {
            follow((shooterMotorA as HardwareSparkMaxDevice).device, true)
        }

        useHardware(turretRotationMotor) {
            idleMode = CANSparkMax.IdleMode.kBrake

            enableVoltageCompensation(12.0)
        }

        turretRotationGearbox.setAngularPosition(0.0.Revolutions)

        turretController.setTolerance(1.0.Degrees.toRadians().value)

        on (Events.ENABLED) {
            //flywheelMachine.setState(FlywheelStates.PreSpin)
        }

        SmartDashboard.putNumber("shooter_p", 0.0)

        on (Events.TELEOP_ENABLED) {
            turretMachine.setState(TurretStates.LockToZero)
            //flywheelMachine.setState(FlywheelStates.PreSpin)
        }
    }
}