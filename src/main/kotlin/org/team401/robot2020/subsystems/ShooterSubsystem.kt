package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrame
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
import org.snakeskin.component.impl.HardwareTalonFxDevice
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.component.impl.NullTalonFxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRevolutionsPerSecond
import org.snakeskin.utility.Ticker
import org.snakeskin.utility.value.AsyncValue
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.DIOChannels
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.control.VelocityProfiler
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.robot.TurretLimelight
import org.team401.robot2020.util.CharacterizationRoutine
import org.team401.robot2020.util.LoggerSession
import org.team401.robot2020.util.getAcceleration
import org.team401.taxis.geometry.Rotation2d

object ShooterSubsystem: Subsystem() {
    //<editor-fold desc="Hardware Devices">

    private val shooterMotorA = Hardware.createTalonFX(
        CANDevices.shooterMotorA.canID,
        mockProducer = NullTalonFxDevice.producer
    )

    private val shooterMotorB = Hardware.createTalonFX(
        CANDevices.shooterMotorA.canID,
        mockProducer = NullTalonFxDevice.producer
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
    //</editor-fold>

    //<editor-fold desc="Models and Controllers">
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
    //</editor-fold>

    fun getTurretAngle(): Rotation2d {
        return Rotation2d.fromRadians(turretRotationGearbox.getAngularPosition().toRadians().value)
    }

    enum class TurretStates {
        Homing, //Turret is homing
        Hold, //Turret is holding its current position
        LockToZero, //Turret is pointed straight ahead
        LockForClimb, //Turret goes to the climbing position
        Seeking, //Turret is looking for the vision target on the wall
        FollowingTarget //Turret is locked onto the vision target
    }

    enum class FlywheelStates {
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
                kickerMotor.setPercentOutput(1.0)
            }
        }

        disabled {
            action {
                kickerMotor.stop()
            }
        }
    }

    val turretMachine: StateMachine<TurretStates> = stateMachine {
        state(TurretStates.Homing) {

        }

        state(TurretStates.Hold) {
            var position = 0.0.Radians

            entry {
                resetTurret()
                position = turretRotationGearbox.getAngularPosition().toRadians()
            }

            rtAction { timestamp, dt ->
                updateTurret(position)
            }
        }

        state(TurretStates.LockToZero) {
            entry {
                resetTurret()
            }

            rtAction { timestamp, dt ->
                updateTurret(0.0.Radians)
            }
        }

        state(TurretStates.LockForClimb) {
            entry {
                resetTurret()
            }

            rtAction { timestamp, dt ->
                updateTurret((-180.0).Degrees.toRadians())
            }
        }

        state(TurretStates.Seeking) {
            val lockTicker = Ticker({ turretController.atGoal() && TurretLimelight.hasTarget() }, 0.05.Seconds, actionRate)

            entry {
                resetTurret()
                lockTicker.reset()
            }

            rtAction { timestamp, dt ->
                val fieldToVehicle = RobotState.getFieldToVehicle(timestamp)
                val targetRotation = Rotation2d(fieldToVehicle.rotation.inverse().radians, true)

                updateTurret(targetRotation.radians.Radians)
                lockTicker.check {
                    setState(TurretStates.FollowingTarget)
                }
            }
        }

        state(TurretStates.FollowingTarget) {
            val unlockTicker = Ticker({ !TurretLimelight.hasTarget() }, 0.1.Seconds, actionRate)

            entry {
                resetTurret()
            }

            rtAction { timestamp, dt ->
                val aimingParams = RobotState.getTurretAimingParameters(timestamp)
                val targetAngle = Rotation2d(aimingParams.desiredAngle.radians, true)

                updateTurret(targetAngle.radians.Radians)
                unlockTicker.check {
                    setState(TurretStates.Seeking)
                }
            }
        }

        disabled {
            action {
                turretRotationGearbox.stop()
            }
        }
    }


    override fun setup() {
        useHardware(shooterMotorA) {
            inverted = false
            setNeutralMode(NeutralMode.Coast)
            configOpenloopRamp(0.0)
            configNeutralDeadband(0.0)
            setControlFramePeriod(ControlFrame.Control_3_General, 5)
            setStatusFramePeriod(StatusFrame.Status_1_General, 5)
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0)
        }

        useHardware(shooterMotorB) {
            setInverted(InvertType.OpposeMaster)
            setNeutralMode(NeutralMode.Coast)
            setStatusFramePeriod(StatusFrame.Status_1_General, Int.MAX_VALUE)
            setStatusFramePeriod(StatusFrame.Status_2_Feedback0, Int.MAX_VALUE)
        }

        useHardware(turretRotationMotor) {
            idleMode = CANSparkMax.IdleMode.kBrake
            enableVoltageCompensation(12.0)
        }

        turretRotationGearbox.setAngularPosition(0.0.Radians)
        turretController.setTolerance(2.0.Degrees.toRadians().value)

        on (Events.ENABLED) {
            turretMachine.setState(TurretStates.Hold)
        }
    }
}