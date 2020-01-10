package org.team401.robot2020.subsystems
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.component.SmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.hid.channel.AxisChannel
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2020.HumanControllers
import org.team401.robot2020.config.Geometry
import org.team401.robot2020.config.HardwareMap

object DrivetrainSubsystem : Subsystem(), IDifferentialDrivetrain {
    override val geometry = Geometry.DrivetrainGeometry
    override val headingSensor = Hardware.createCANPigeonIMU(HardwareMap.DrivetrainMap.pigeonId)

    private val leftMaster = Hardware.createTalonFX(HardwareMap.DrivetrainMap.leftFrontFalconId)
    private val leftSlave = Hardware.createTalonFX(HardwareMap.DrivetrainMap.leftRearFalconId)
    private val rightMaster = Hardware.createTalonFX(HardwareMap.DrivetrainMap.rightFrontFaconId)
    private val rightSlave = Hardware.createTalonFX(HardwareMap.DrivetrainMap.rightRearFalconId)

    override val left = SmartGearbox(leftMaster, leftSlave)
    override val right = SmartGearbox(rightMaster, rightSlave)

    private val cheesyDriveController = CheesyDriveController()

    enum class States {
        OperatorControl
    }

    val driveMachine: StateMachine<States> = stateMachine {
        state(States.OperatorControl) {
            entry {
                cheesyDriveController.reset()
            }

            action {
                val trans = HumanControllers.driveTranslationChannel.read()
                val rot = HumanControllers.driveRotationChannel.read()
                val output = cheesyDriveController.update(trans, rot, true, HumanControllers.driveQuickTurnChannel.read())
                tank(output.left, output.right)
            }
        }
    }

    override fun setup() {
        on (Events.TELEOP_ENABLED) {
            driveMachine.setState(States.OperatorControl)
        }
    }
}