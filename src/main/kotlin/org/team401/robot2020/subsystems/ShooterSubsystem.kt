package org.team401.robot2020.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.component.CTREFeedforwardScalingMode
import org.snakeskin.component.Gearbox
import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.component.impl.NullTalonFxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.robot2020.HumanControllers
import org.team401.robot2020.config.CANDevices
import org.team401.robot2020.config.DIOChannels
import org.team401.robot2020.config.constants.RobotConstants
import org.team401.robot2020.config.constants.ShooterConstants
import org.team401.robot2020.control.VelocityProfiler
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.turret.TurretController
import org.team401.robot2020.util.*
import org.team401.taxis.geometry.Rotation2d

object ShooterSubsystem: Subsystem() {
    //<editor-fold desc="Hardware Devices">
    private val flywheelMotorA = Hardware.createTalonFX(
        CANDevices.shooterMotorA.canID,
        ffMode = CTREFeedforwardScalingMode.Scale12V,
        mockProducer = NullTalonFxDevice.producer
    )

    private val flywheelMotorB = Hardware.createTalonFX(
        CANDevices.shooterMotorB.canID,
        mockProducer = NullTalonFxDevice.producer
    )

    private val flywheelGearbox = SmartGearbox(flywheelMotorA, flywheelMotorB, ratioToSensor = ShooterConstants.flywheelRatio)

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

    private val hoodPiston = Hardware.createPneumaticChannel(0)
    //</editor-fold>

    //<editor-fold desc="Models and Controllers">
    private val flywheelProfiler = VelocityProfiler(ShooterConstants.flywheelRampAcceleration)
    private val flywheelModel = SimpleMotorFeedforward(
        ShooterConstants.flywheelKs,
        ShooterConstants.flywheelKv,
        ShooterConstants.flywheelKa
    )

    private val turretController = TurretController(
        ShooterConstants.turretKp,
        ShooterConstants.turretKd,
        ShooterConstants.turretTrackingKp,
        ShooterConstants.turretTrackingKd,
        ShooterConstants.turretKs,
        ShooterConstants.turretKv,
        ShooterConstants.turretKa,
        ShooterConstants.turretVelocity,
        ShooterConstants.turretAccel,
        ShooterConstants.turretRapidError,
        ShooterConstants.turretNegativeLimit,
        ShooterConstants.turretPositiveLimit,
        turretRotationGearbox,
        turretRotationGearbox,
        RobotConstants.rtPeriod
    )

    val turretCharacterizationRoutine = CharacterizationRoutine(turretRotationGearbox, turretRotationGearbox)
    val flywheelCharacterizationRoutine = CharacterizationRoutine(flywheelGearbox, flywheelGearbox)
    //</editor-fold>

    fun getTurretAngle(): AngularDistanceMeasureRadians {
        return turretRotationGearbox.getAngularPosition()
    }

    fun getTurretVelocity(): AngularVelocityMeasureRadiansPerSecond {
        return turretRotationGearbox.getAngularVelocity()
    }

    fun getFlywheelVelocity(): AngularVelocityMeasureRadiansPerSecond {
        return flywheelGearbox.getAngularVelocity()
    }

    fun isTurretInRapid(): Boolean {
        return turretController.isInRapid
    }

    fun isShotReady(): Boolean {
        return flywheelProfiler.goal > 0.0.RadiansPerSecond && flywheelGearbox.getAngularVelocity().toRevolutionsPerMinute() > flywheelProfiler.goal.toRevolutionsPerMinute() - 20.0.RevolutionsPerMinute
    }

    fun setHoodState(farShot: Boolean) {
        hoodPiston.setState(farShot)
    }

    private var flywheelAdjustmentNear = 0.0.RadiansPerSecond
        @Synchronized get

    private var flywheelAdjustmentFar = 0.0.RadiansPerSecond
        @Synchronized get

    @Synchronized fun adjustFlywheelUp() {
        when (flywheelMachine.getState()) {
            FlywheelStates.NearShotSpinUp -> flywheelAdjustmentNear += ShooterConstants.flywheelBump
            FlywheelStates.FarShotSpinUp -> flywheelAdjustmentFar += ShooterConstants.flywheelBump
        }
    }

    @Synchronized fun adjustFlywheelDown() {
        when (flywheelMachine.getState()) {
            FlywheelStates.NearShotSpinUp -> flywheelAdjustmentNear -= ShooterConstants.flywheelBump
            FlywheelStates.FarShotSpinUp -> flywheelAdjustmentFar -= ShooterConstants.flywheelBump
        }
    }

    @Synchronized fun resetFlywheelAdjust() {
        flywheelAdjustmentNear = 0.0.RadiansPerSecond
        flywheelAdjustmentFar = 0.0.RadiansPerSecond
    }

    private fun updateFlywheel(dt: TimeMeasureSeconds, velocity: AngularVelocityMeasureRadiansPerSecond) {
        flywheelProfiler.calculate(dt, velocity)
        val ffVolts = flywheelModel.calculate(
            flywheelProfiler.velocityCommand.value,
            flywheelProfiler.accelerationCommand.value
        )

        flywheelGearbox.setAngularVelocitySetpoint(flywheelProfiler.velocityCommand, ffVolts)
    }

    enum class TurretStates {
        Homing, //Turret is homing
        Hold, //Turret is holding its current position
        LockToZero, //Turret is pointed straight ahead
        LockForClimb, //Turret goes to the climbing position

        FieldRelativeTarget, //Turret is pointed at the last known location of the target on the field
        Jogging, //Turret is being jogged manually
    }

    enum class FlywheelStates {
        NearShotSpinUp, //Spins up to the near shot speed based on the targeting info
        FarShotSpinUp, //Spins up to the far shot speed based on the targeting info
        HoldSetpoint, //Holds the present setpoint.  Must be entered from a spin-up state
        Manual //Flywheel is under manual, OPEN LOOP control
    }

    enum class KickerStates {
        Kick
    }

    val flywheelMachine: StateMachine<FlywheelStates> = stateMachine {
        state(FlywheelStates.NearShotSpinUp) {
            entry { flywheelProfiler.reset(flywheelGearbox.getAngularVelocity()) }

            rtAction { timestamp, dt ->
                val targetingParameters = RobotState.getTargetingParameters(timestamp, ShooterConstants.turretTrackingLookahead)
                val shotVelocity = (ShooterConstants.flywheelRegression
                    .predict(targetingParameters.rangeToTarget.value).RadiansPerSecond + flywheelAdjustmentNear + ShooterConstants.flywheelNearConstantCorrection)
                    .coerceAtLeast(ShooterConstants.flywheelMinimumSpeed) //If adjustment is too low, enforce min speed

                updateFlywheel(dt, shotVelocity)
            }
        }

        state(FlywheelStates.FarShotSpinUp) {
            entry { flywheelProfiler.reset(flywheelGearbox.getAngularVelocity()) }
            
            rtAction { _, dt ->
                val shotVelocity = (ShooterConstants.flywheelFarShotSpeed + flywheelAdjustmentFar + ShooterConstants.flywheelFarConstantCorrection)
                    .coerceAtLeast(ShooterConstants.flywheelMinimumSpeed)

                updateFlywheel(dt, shotVelocity)
            }
        }

        state(FlywheelStates.HoldSetpoint) {
            var velocity = ShooterConstants.flywheelMinimumSpeed
            rejectIf { !isInState(FlywheelStates.NearShotSpinUp) && !isInState(FlywheelStates.FarShotSpinUp) }

            entry {
                velocity = flywheelProfiler.goal
            }

            rtAction { _, dt ->
                updateFlywheel(dt, velocity)
            }
        }

        state(FlywheelStates.Manual) {
            action {
                flywheelGearbox.setPercentOutput(HumanControllers.manualShotPowerChannel.read())
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
            entry { turretController.enterHold() }
            rtAction { _, _ -> turretController.updateHold() }
        }

        state(TurretStates.LockToZero) {
            entry { turretController.enterAbsoluteAngle() }
            rtAction { _, _ -> turretController.updateAbsoluteAngle(0.0.Radians) }
        }

        state(TurretStates.LockForClimb) {
            entry { turretController.enterAbsoluteAngle() }
            rtAction { _, _ -> turretController.updateAbsoluteAngle((-180.0).Degrees.toRadians()) }
        }

        state(TurretStates.FieldRelativeTarget) {
            entry { turretController.enterAngle() }

            rtAction { timestamp, _ ->
                val aimingParameters = RobotState.getTargetingParameters(timestamp, ShooterConstants.turretTrackingLookahead)
                val targetAngle = Rotation2d.fromRadians(turretRotationGearbox.getAngularPosition().value + aimingParameters.turretError.radians)

                turretController.updateAngle(targetAngle, aimingParameters.turretFeedVelocity)
            }
        }

        state(TurretStates.Jogging) {
            entry { turretController.enterJog() }

            rtAction { _, dt ->
                val jogRate = HumanControllers.turretJogChannel.read()._ul * ShooterConstants.turretJogRate
                turretController.updateJog(jogRate, dt)
            }
        }

        disabled {
            entry {
                flywheelProfiler.reset(0.0.RadiansPerSecond)
                turretRotationGearbox.stop()
            }
        }
    }

    override fun setup() {
        useHardware(flywheelMotorA) {
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

        useHardware(flywheelMotorB) {
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

        turretRotationGearbox.setAngularPosition(0.0.Radians)

        on (Events.ENABLED) {
            turretMachine.setState(TurretStates.Hold)
            setHoodState(false)
        }

        on (Events.DISABLED) {
            //Print out flywheel adjustments on disabled for logging purposes.
            println("FLYWHEEL NEAR ADJUST: ${flywheelAdjustmentNear.toRevolutionsPerMinute()}")
            println("FLYWHEEL FAR ADJUST: ${flywheelAdjustmentFar.toRevolutionsPerMinute()}")
        }
    }
}