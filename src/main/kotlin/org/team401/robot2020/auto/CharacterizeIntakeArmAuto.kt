package org.team401.robot2020.auto

import edu.wpi.first.networktables.NetworkTableInstance
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.measure.Revolutions
import org.snakeskin.measure.RevolutionsPerSecond
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.runtime.SnakeskinRuntime
import org.team401.robot2020.subsystems.BallSubsystem
import kotlin.math.absoluteValue

class CharacterizeIntakeArmAuto: RobotAuto(0.01.Seconds, 0.0.Seconds) {
    private inner class Step: AutoStep() {
        private var prevCommand = 0.0
        private val arr = DoubleArray(10)

        private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
        private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

        override fun entry(currentTime: TimeMeasureSeconds) {
            prevCommand = 0.0
            NetworkTableInstance.getDefault().setUpdateRate(0.01)
        }

        override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds): Boolean {
            //Read left pos and vel (rad, rad/s)
            val position = BallSubsystem.intakePivotGearbox.getAngularPosition().toRadians()
            val velocity = BallSubsystem.intakePivotGearbox.getAngularVelocity().toRadiansPerSecond()

            //Read voltage and command
            val battery = SnakeskinRuntime.voltage
            val motorVolts = battery * prevCommand.absoluteValue

            //Get new command from NT
            val command = autoSpeedEntry.getDouble(0.0)
            prevCommand = command

            BallSubsystem.intakePivotGearbox.setPercentOutput(command)

            //Send telemetry
            arr[0] = currentTime.value    //Time in seconds
            arr[1] = battery              //Battery voltage
            arr[2] = command              //Applied command
            arr[3] = motorVolts           //Applied voltage (left)
            arr[4] = position.value   //Left position (radians)
            arr[5] = velocity.value  //Right position (radians)

            telemetryEntry.setDoubleArray(arr)

            return false //This step never finishes by itself
        }

        override fun exit(currentTime: TimeMeasureSeconds) {
            //drivetrain.stop()
        }
    }

    override fun assembleAuto() = SequentialSteps(Step())
}
