package org.team401.robot2020.util

import edu.wpi.first.networktables.NetworkTableInstance
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.provider.IAngularPositionProvider
import org.snakeskin.component.provider.IAngularVelocityProvider
import org.snakeskin.component.provider.IPercentOutputMotorControlProvider
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.runtime.SnakeskinRuntime
import kotlin.math.absoluteValue

/**
 * Performs characterization telemetry to the frc-characterization tool for any system but a drive.  Uses radians for all units
 */
class CharacterizationRoutine<I>(val inputProvider: I, val outputProvider: IPercentOutputMotorControlProvider): AutoStep() where I : IAngularPositionProvider, I: IAngularVelocityProvider {
    private var prevCommand = 0.0
    private val arr = DoubleArray(6)

    private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
    private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

    override fun entry(currentTime: TimeMeasureSeconds) {
        prevCommand = 0.0
        NetworkTableInstance.getDefault().setUpdateRate(0.01)
    }

    override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds): Boolean {
        val position = inputProvider.getAngularPosition().toRadians()
        val velocity = inputProvider.getAngularVelocity().toRadiansPerSecond()

        //Read voltage and command
        val battery = SnakeskinRuntime.voltage
        val motorVolts = battery * prevCommand.absoluteValue

        //Get new command from NT
        val command = autoSpeedEntry.getDouble(0.0)
        prevCommand = command

        outputProvider.setPercentOutput(command)

        //Send telemetry
        arr[0] = currentTime.value    //Time in seconds
        arr[1] = battery              //Battery voltage
        arr[2] = command              //Applied command
        arr[3] = motorVolts           //Applied voltage
        arr[4] = position.value       //Position (radians)
        arr[5] = velocity.value       //Velocity (radians/s)

        telemetryEntry.setDoubleArray(arr)

        return false //This step never finishes by itself
    }

    override fun exit(currentTime: TimeMeasureSeconds) {
        outputProvider.stop()
    }

    val loop = object : RobotAuto(0.01.Seconds) {
        override fun assembleAuto(): SequentialSteps {
            return SequentialSteps(this@CharacterizationRoutine)
        }
    }
}