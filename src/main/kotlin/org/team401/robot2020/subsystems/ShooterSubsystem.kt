package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.LinearFilter
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.CTREFeedforwardScalingMode
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
import org.team401.robot2020.control.robot.SuperstructureManager
import org.team401.robot2020.control.robot.TurretLimelight
import org.team401.robot2020.util.CharacterizationRoutine
import org.team401.robot2020.util.LoggerSession
import org.team401.robot2020.util.getAcceleration
import org.team401.taxis.geometry.Rotation2d

object ShooterSubsystem: Subsystem() {
    //<editor-fold desc="Hardware Devices">
    private val shooterMotorA = Hardware.createTalonFX(
        CANDevices.shooterMotorA.canID,
        ffMode = CTREFeedforwardScalingMode.Scale12V,
        mockProducer = NullTalonFxDevice.producer
    )

    private val shooterMotorB = Hardware.createTalonFX(
        CANDevices.shooterMotorB.canID,
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

    private val visionTurretController by lazy { PIDController(
        ShooterConstants.turretTrackingKp, 0.0,
        ShooterConstants.turretTrackingKd,
        0.01
    )}

    private val turretModel = SimpleMotorFeedforward(
        ShooterConstants.turretKs,
        ShooterConstants.turretKv,
        ShooterConstants.turretKa
    )

    private var lastTurretSetpoint = TrapezoidProfile.State()

    val turretCharacterizationRoutine = CharacterizationRoutine(turretRotationGearbox, turretRotationGearbox)
    val flywheelCharacterizationRoutine = CharacterizationRoutine(shooterGearbox, shooterGearbox)
    //</editor-fold>

    fun getTurretAngle(): AngularDistanceMeasureRadians {
        return turretRotationGearbox.getAngularPosition()
    }

    fun getTurretVelocity(): AngularVelocityMeasureRadiansPerSecond {
        return turretRotationGearbox.getAngularVelocity()
    }

    fun isShotReady(): Boolean {
        return true//shooterGearbox.getAngularVelocity().toRevolutionsPerMinute() > flywheelProfiler.velocityCommand.toRevolutionsPerMinute() - 20.0.RevolutionsPerMinute
    }

    fun getTurretRotation(): Rotation2d {
        return Rotation2d.fromRadians(turretRotationGearbox.getAngularPosition().toRadians().value)
    }

    /*
    fun isFlywheelAtShootingSpeed(): Boolean {
        //TODO use the actual speed
        val targetSpeed = ShooterConstants.flywheelDefaultSpeed
        val currentSpeed = shooterGearbox.getAngularVelocity()
    }

     */

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
        val ffVolts = flywheelModel.calculate(
            flywheelProfiler.velocityCommand.value,
            flywheelProfiler.accelerationCommand.value
        )

        shooterGearbox.setAngularVelocitySetpoint(flywheelProfiler.velocityCommand, ffVolts)
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

    private fun visionUpdateTurret(target: AngularDistanceMeasureRadians, vel: AngularVelocityMeasureRadiansPerSecond) {
        val feedbackVolts = visionTurretController.calculate(turretRotationGearbox.getAngularPosition().toRadians().value, target.value)
        val ffVolts = turretModel.calculate(vel.value)
        turretRotationGearbox.setPercentOutput((feedbackVolts + ffVolts) / 12.0)
    }

    val flywheelMachine: StateMachine<FlywheelStates> = stateMachine {
        state(FlywheelStates.PreSpin) {
            lateinit var session: LoggerSession

            entry {
                session = LoggerSession("10.4.1.162", 5801)
                flywheelProfiler.reset(shooterGearbox.getAngularVelocity().toRadiansPerSecond())
            }

            rtAction { timestamp, dt ->
                updateFlywheel(dt, ShooterConstants.flywheelRegression.predict(SuperstructureManager.distanceFromTarget.value).RevolutionsPerMinute.toRadiansPerSecond())
                //updateFlywheel(dt, SmartDashboard.getNumber("flywheel_rpm", 0.0).RevolutionsPerMinute.toRadiansPerSecond())
                session["Timestamp (s)"] = timestamp.value
                session["Velocity Command (RPM)"] = flywheelProfiler.velocityCommand.toRevolutionsPerMinute().value
                session["Velocity (RPM)"] = shooterGearbox.getAngularVelocity().toRevolutionsPerMinute().value
                session.publish()
            }

            exit {
                session.end()
            }
        }

        state(FlywheelStates.CoordinatedControl) {
            entry {
                flywheelProfiler.reset(shooterGearbox.getAngularVelocity().toRadiansPerSecond())
            }

            rtAction { timestamp, dt ->
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
            entry {
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
                turretRotationGearbox.stop()
                //updateTurret(position)
            }
        }

        state(TurretStates.LockToZero) {
            entry {
                resetTurret()
            }

            rtAction { timestamp, dt ->
                updateTurret(ShooterConstants.turretZeroOffset)
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
            //val unlockTicker = Ticker({ !TurretLimelight.hasTarget() }, 0.1.Seconds, actionRate)

            entry {
                resetTurret()
                //filter.reset()
                //RobotState.resetVision()
            }

            rtAction { timestamp, dt ->
                val targetPosition = SuperstructureManager.turretPositionSetpoint
                val targetVelocity = SuperstructureManager.turretVelocitySetpoint

                visionUpdateTurret(targetPosition, targetVelocity) //Command the angle determined from the operation above
            }
        }

        disabled {
            entry {
                turretRotationGearbox.stop()
            }
        }
    }

    override fun setup() {
        useHardware(shooterMotorA) {
            inverted = false
            setNeutralMode(NeutralMode.Coast)
            configOpenloopRamp(0.0)
            configClosedloopRamp(0.0)
            configNeutralDeadband(0.0)
            configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000)
            setControlFramePeriod(ControlFrame.Control_3_General, 5)
            setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, 1000)
            setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 1000)
            configVelocityMeasurementWindow(1, 1000)
            configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 1000)

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
            setSmartCurrentLimit(20)
            enableVoltageCompensation(12.0)
        }

        useHardware(kickerMotor) {
            setSmartCurrentLimit(20)
        }

        turretRotationGearbox.setAngularPosition(ShooterConstants.turretZeroOffset)
        turretController.setTolerance(2.0.Degrees.toRadians().value)

        on (Events.ENABLED) {
            turretMachine.setState(TurretStates.Hold)
        }

        SmartDashboard.putNumber("flywheel_rpm", 0.0)
    }
}