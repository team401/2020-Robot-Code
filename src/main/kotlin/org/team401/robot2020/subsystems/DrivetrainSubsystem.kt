package org.team401.robot2020.subsystems
import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrame
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import org.snakeskin.component.Gearbox
import org.snakeskin.component.SmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.logic.scalars.LowPassScalar
import org.snakeskin.measure.*
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2020.HumanControllers
import org.team401.robot2020.config.DrivetrainGeometry
import org.team401.robot2020.config.DrivetrainDynamics
import org.team401.robot2020.config.HardwareMap
import org.team401.robot2020.control.robot.RobotState
import org.team401.taxis.diffdrive.component.IModeledDifferentialDrivetrain
import org.team401.taxis.diffdrive.component.impl.YawHeadingSource
import org.team401.taxis.diffdrive.component.provider.IHeadingProvider
import org.team401.taxis.diffdrive.control.DifferentialDrivetrainModel
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.geometry.Pose2d

object DrivetrainSubsystem : Subsystem(), IModeledDifferentialDrivetrain {
    override val geometry = DrivetrainGeometry
    override val model = DifferentialDrivetrainModel(DrivetrainGeometry, DrivetrainDynamics)

    private val leftModelOpenLoop = SimpleMotorFeedforward(
        DrivetrainDynamics.leftKs,
        DrivetrainDynamics.leftKv
    )

    private val rightModelOpenLoop = SimpleMotorFeedforward(
        DrivetrainDynamics.rightKs,
        DrivetrainDynamics.rightKv
    )

    private val leftPid = PIDController(DrivetrainDynamics.driveLeftKp, 0.0, 0.0)
    private val rightPid = PIDController(DrivetrainDynamics.driveRightKp, 0.0, 0.0)

    val pathManager = DrivetrainPathManager(
        model,
        NonlinearFeedbackPathController(2.0, .7),
        2.0, .25, 5.0.Degrees.toRadians().value
    )

    override val yawSensor = Hardware.createCANPigeonIMU(HardwareMap.DrivetrainMap.pigeonId)
    override val headingSource = YawHeadingSource(yawSensor)

    override val driveState = RobotState

    private val leftMaster = Hardware.createTalonFX(HardwareMap.DrivetrainMap.leftFrontFalconId)
    private val leftSlave = Hardware.createTalonFX(HardwareMap.DrivetrainMap.leftRearFalconId)
    private val rightMaster = Hardware.createTalonFX(HardwareMap.DrivetrainMap.rightFrontFaconId)
    private val rightSlave = Hardware.createTalonFX(HardwareMap.DrivetrainMap.rightRearFalconId)

    private val leftEncoder = Hardware.createDIOEncoder(
        HardwareMap.DrivetrainMap.leftEncoderA,
        HardwareMap.DrivetrainMap.leftEncoderB,
        8192.0,
        false
    )

    private val rightEncoder = Hardware.createDIOEncoder(
        HardwareMap.DrivetrainMap.rightEncoderA,
        HardwareMap.DrivetrainMap.rightEncoderB,
        8192.0,
        true
    )

    override val left = Gearbox(leftEncoder, leftMaster, leftSlave)
    override val right = Gearbox(rightEncoder, rightMaster, rightSlave)

    private val cheesyDriveController = CheesyDriveController()

    enum class States {
        OperatorControl,
        TrajectoryFollowing
    }

    val driveMachine: StateMachine<States> = stateMachine {
        state(States.OperatorControl) {
            entry {
                cheesyDriveController.reset()
                configForTeleopDriving()
            }

            action {
                val trans = HumanControllers.driveTranslationChannel.read()
                val rot = HumanControllers.driveRotationChannel.read()
                val output = cheesyDriveController.update(trans, rot, true, HumanControllers.driveQuickTurnChannel.read())
                val leftVel = output.left._ul * (DrivetrainDynamics.driveSpeedIdeal.toAngularVelocity(geometry.wheelRadius))
                val rightVel = output.right._ul * (DrivetrainDynamics.driveSpeedIdeal.toAngularVelocity(geometry.wheelRadius))
                val leftOut = leftModelOpenLoop.calculate(leftVel.value) / 12.0
                val rightOut = rightModelOpenLoop.calculate(rightVel.value) / 12.0
                tank(leftOut, rightOut)
            }
        }

        state(States.TrajectoryFollowing) {
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

    override fun setup() {
        configForStartup()

        setPose(Pose2d.identity(), readTimestamp())

        on (Events.TELEOP_ENABLED) {
            driveMachine.setState(States.OperatorControl)
        }
    }
}