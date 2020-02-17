package org.team401.robot2020.subsystems
import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrame
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.component.Gearbox
import org.snakeskin.component.impl.NullPigeonImuDevice
import org.snakeskin.component.impl.NullTalonFxDevice
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.*
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2020.HumanControllers
import org.team401.robot2020.config.*
import org.team401.robot2020.config.constants.DrivetrainConstants
import org.team401.robot2020.control.robot.RobotState
import org.team401.taxis.diffdrive.component.IModeledDifferentialDrivetrain
import org.team401.taxis.diffdrive.component.impl.YawHeadingSource
import org.team401.taxis.diffdrive.control.DifferentialDrivetrainModel
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.geometry.Pose2d

object DrivetrainSubsystem : Subsystem(), IModeledDifferentialDrivetrain {
    //<editor-fold desc="Hardware Devices">
    private val leftMaster = Hardware.createTalonFX(
        CANDevices.driveLeftFrontMotor.canID,
        mockProducer = NullTalonFxDevice.producer
    )
    private val leftSlave = Hardware.createTalonFX(
        CANDevices.driveLeftRearMotor.canID,
        mockProducer = NullTalonFxDevice.producer
    )
    private val rightMaster = Hardware.createTalonFX(
        CANDevices.driveRightFrontMotor.canID,
        mockProducer = NullTalonFxDevice.producer
    )
    private val rightSlave = Hardware.createTalonFX(
        CANDevices.driveRightRearMotor.canID,
        mockProducer = NullTalonFxDevice.producer
    )

    override val yawSensor = Hardware.createCANPigeonIMU(
        CANDevices.pigeon,
        mockProducer = NullPigeonImuDevice.producer
    )
    override val headingSource = YawHeadingSource(yawSensor)

    private val leftEncoder = Hardware.createDIOEncoder(
        DIOChannels.driveLeftEncoderA,
        DIOChannels.driveLeftEncoderB,
        8192.0,
        false,
        halMock = true
    )

    private val rightEncoder = Hardware.createDIOEncoder(
        DIOChannels.driveRightEncoderA,
        DIOChannels.driveRightEncoderB,
        8192.0,
        true,
        halMock = true
    )

    override val left = Gearbox(leftEncoder, leftMaster, leftSlave)
    override val right = Gearbox(rightEncoder, rightMaster, rightSlave)
    //</editor-fold>

    //<editor-fold desc="Models and Controllers">
    override val geometry = DrivetrainConstants
    override val model = DifferentialDrivetrainModel(
        DrivetrainConstants,
        DrivetrainConstants
    )

    private val leftSimpleModel = SimpleMotorFeedforward(
        DrivetrainConstants.leftKs,
        DrivetrainConstants.leftKv
    )

    private val rightSimpleModel = SimpleMotorFeedforward(
        DrivetrainConstants.rightKs,
        DrivetrainConstants.rightKv
    )

    //Lazy init since these make HAL calls for usage reporting (for some stupid reason)
    private val leftPid by lazy { PIDController(DrivetrainConstants.leftKp, 0.0, 0.0) }
    private val rightPid by lazy { PIDController(DrivetrainConstants.rightKp, 0.0, 0.0) }

    val pathManager = DrivetrainPathManager(model, NonlinearFeedbackPathController())
    override val driveState = RobotState

    private val cheesyDriveController = CheesyDriveController(DrivetrainConstants.CheesyDriveParameters)
    //</editor-fold>

    fun getRoll() = yawSensor.getRoll()

    enum class DriveStates {
        OperatorControl,
        TrajectoryFollowing
    }

    val driveMachine: StateMachine<DriveStates> = stateMachine {
        state(DriveStates.OperatorControl) {
            entry {
                cheesyDriveController.reset()
                configForTeleopDriving()
            }

            action {
                val trans = HumanControllers.driveTranslationChannel.read()
                val rot = HumanControllers.driveRotationChannel.read()
                val output = cheesyDriveController.update(trans, rot, false, HumanControllers.driveQuickTurnChannel.read())
                val leftVel = output.left._ul * DrivetrainConstants.maxSpeed
                val rightVel = output.right._ul * DrivetrainConstants.maxSpeed
                val leftOut = leftSimpleModel.calculate(leftVel.value) / 12.0
                val rightOut = rightSimpleModel.calculate(rightVel.value) / 12.0
                tank(leftOut, rightOut)
            }
        }

        state(DriveStates.TrajectoryFollowing) {
            entry {
                configForAutoDriving()
            }

            rtAction { timestamp, _ ->
                val output = pathManager.update(timestamp.value, driveState.getFieldToVehicle(timestamp))

                val leftVelocity = left.getAngularVelocity().toRadiansPerSecond()
                val rightVelocity = right.getAngularVelocity().toRadiansPerSecond()

                val leftFf = output.left_feedforward_voltage / 12.0
                val rightFf = output.right_feedforward_voltage / 12.0
                val leftFeedback = leftPid.calculate(leftVelocity.value, output.left_velocity) / 12.0
                val rightFeedback = rightPid.calculate(rightVelocity.value, output.right_velocity) / 12.0

                tank(leftFf + leftFeedback, rightFf + rightFeedback)
            }
        }

        disabled {
            action {
                stop()
            }
        }
    }

    override fun action() {
        //println("L: ${left.getAngularPosition()}  R: ${right.getAngularPosition()}")
        //println(driveState.getLatestFieldToVehicle().value)
    }

    //<editor-fold desc="Motor Controller Configuration Functions">
    private fun configForStartup() {
        left.couple()
        right.couple()

        useHardware(leftMaster) {
            inverted = true
            setNeutralMode(NeutralMode.Brake)
            configOpenloopRamp(0.0)
            configNeutralDeadband(0.0)
            setControlFramePeriod(ControlFrame.Control_3_General, 5)
            setStatusFramePeriod(StatusFrame.Status_1_General, 5)
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0)
        }
        useHardware(leftSlave) {
            setInverted(InvertType.FollowMaster)
            setNeutralMode(NeutralMode.Brake)
            setStatusFramePeriod(StatusFrame.Status_1_General, Int.MAX_VALUE)
            setStatusFramePeriod(StatusFrame.Status_2_Feedback0, Int.MAX_VALUE)
        }
        useHardware(rightMaster) {
            inverted = false
            setNeutralMode(NeutralMode.Brake)
            configOpenloopRamp(0.0)
            configNeutralDeadband(0.0)
            setControlFramePeriod(ControlFrame.Control_3_General, 5)
            setStatusFramePeriod(StatusFrame.Status_1_General, 5)
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0)
        }
        useHardware(rightSlave) {
            setInverted(InvertType.FollowMaster)
            setNeutralMode(NeutralMode.Brake)
            setStatusFramePeriod(StatusFrame.Status_1_General, Int.MAX_VALUE)
            setStatusFramePeriod(StatusFrame.Status_2_Feedback0, Int.MAX_VALUE)
        }
    }

    private fun configForAutoDriving() {
        useHardware(leftMaster) {
            configOpenloopRamp(0.0)
            configNeutralDeadband(0.0)
        }

        useHardware(rightMaster) {
            configOpenloopRamp(0.0)
            configNeutralDeadband(0.0)
        }
    }

    private fun configForTeleopDriving() {
        useHardware(leftMaster) {
            configOpenloopRamp(0.25)
            configNeutralDeadband(0.05)
        }

        useHardware(rightMaster) {
            configOpenloopRamp(0.25)
            configNeutralDeadband(0.05)
        }
    }
    //</editor-fold>

    override fun setup() {
        configForStartup()
        setPose(Pose2d.identity(), readTimestamp())

        on (Events.TELEOP_ENABLED) {
            driveMachine.setState(DriveStates.OperatorControl)
        }
    }
}