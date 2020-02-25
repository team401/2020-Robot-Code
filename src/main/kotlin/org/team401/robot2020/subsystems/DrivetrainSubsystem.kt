package org.team401.robot2020.subsystems
import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.component.Gearbox
import org.snakeskin.component.impl.NullPigeonImuDevice
import org.snakeskin.component.impl.NullSparkMaxDevice
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
    private val leftMaster = Hardware.createBrushlessSparkMax(
        CANDevices.driveLeftFrontMotor.canID,
        mockProducer = NullSparkMaxDevice.producer
    )
    private val leftSlave = Hardware.createBrushlessSparkMax(
        CANDevices.driveLeftRearMotor.canID,
        mockProducer = NullSparkMaxDevice.producer
    )
    private val rightMaster = Hardware.createBrushlessSparkMax(
        CANDevices.driveRightFrontMotor.canID,
        mockProducer = NullSparkMaxDevice.producer
    )
    private val rightSlave = Hardware.createBrushlessSparkMax(
        CANDevices.driveRightRearMotor.canID,
        mockProducer = NullSparkMaxDevice.producer
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
            setIdleMode(CANSparkMax.IdleMode.kBrake)
            setOpenLoopRampRate(0.0)
            enableVoltageCompensation(12.0)

        }
        useHardware(leftSlave) {

        }
        useHardware(rightMaster) {
            inverted = false
            setIdleMode(CANSparkMax.IdleMode.kBrake)
            setOpenLoopRampRate(0.0)
            enableVoltageCompensation(12.0)
        }
        useHardware(rightSlave) {

        }
    }

    private fun configForAutoDriving() {
        useHardware(leftMaster) {
            setOpenLoopRampRate(0.0)
        }

        useHardware(rightMaster) {
            setOpenLoopRampRate(0.0)
        }
    }

    private fun configForTeleopDriving() {
        useHardware(leftMaster) {
            setOpenLoopRampRate(.25)

        }

        useHardware(rightMaster) {
            setOpenLoopRampRate(.25)

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