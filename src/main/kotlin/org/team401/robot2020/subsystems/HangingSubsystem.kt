package org.team401.robot2020.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.snakeskin.component.Gearbox
import org.snakeskin.component.SmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Inches
import org.snakeskin.measure.InchesPerSecond
import org.snakeskin.measure.InchesPerSecondPerSecond
import org.snakeskin.measure.Seconds
import org.team401.robot2020.config.HangerDynamics
import org.team401.robot2020.config.HangerParameters
import org.team401.robot2020.config.HardwareMap

object HangingSubsystem: Subsystem() {
    /*private val rightSpark = Hardware.createBrushlessSparkMax(HardwareMap.HangingMap.rightSparkId)
    private val leftSpark = Hardware.createBrushlessSparkMax(HardwareMap.HangingMap.leftSparkId)

    private val rightCanCoder = Hardware.createCANCoder(HardwareMap.HangingMap.rightCanCoderId)
    private val leftCanCoder = Hardware.createCANCoder(HardwareMap.HangingMap.leftCanCoderId)

    private val rightClimberGearbox = Gearbox(rightCanCoder, rightSpark)
    private val leftClimberGearbox = Gearbox(leftCanCoder, leftSpark)

    private val hangerModel = SimpleMotorFeedforward(HangerDynamics.kS, HangerDynamics.kV)

    private val hangerPID = ProfiledPIDController(
        HangerDynamics.kP,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(HangerParameters.maxVelocity.value,
            HangerParameters.maxAccel.value
        )
    )

    enum class States {
        Extended,
        Retracted,
        Disabled
    }

    val HangingMachine: StateMachine<States> = stateMachine() {
        val desiredExtendedDistance = 24.0.Inches
        val desiredRetractedDistance = 0.0.Inches



        state(States.Extended) {
            hangerPID.goal

        }

        state(States.Retracted) {

        }
    }*/

    private val spark = Hardware.createBrushlessSparkMax(26)



    override fun setup() {
        useHardware(spark) {
            inverted = false
        }
        on(Events.TELEOP_ENABLED) {
            //HangingMachine.setState(States.Disabled)
            spark.setPercentOutput(0.25)
        }
    }
}