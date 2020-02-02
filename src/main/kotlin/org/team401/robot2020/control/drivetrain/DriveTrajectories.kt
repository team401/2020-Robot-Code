package org.team401.robot2020.control.drivetrain

import edu.wpi.first.hal.JNIWrapper
import org.snakeskin.init.InitManager
import org.snakeskin.measure.Degrees
import org.snakeskin.runtime.SnakeskinPlatform
import org.team401.robot2020.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.control.DifferentialDrivetrainModel
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.diffdrive.visualization.visualize
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Twist2d
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimedState
import org.team401.taxis.trajectory.timing.TimingConstraint
import java.awt.*
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import java.lang.RuntimeException
import java.text.DecimalFormat
import java.util.concurrent.ScheduledThreadPoolExecutor
import java.util.concurrent.ThreadFactory
import java.util.concurrent.TimeUnit
import javax.swing.*
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.roundToInt
import kotlin.math.roundToLong
import kotlin.system.exitProcess

object DriveTrajectories {
    val pathManager = DrivetrainSubsystem.pathManager

    val testTrajectory = pathManager.generateTrajectory(
        false,
        listOf(
            Pose2d(24.0, 240.0, Rotation2d.fromDegrees(90.0)),
            Pose2d(24.0 + 120.0, 240.0 + 120.0, Rotation2d.fromDegrees(0.0)),
            Pose2d(24.0 + 120.0 + 120.0, 240.0, Rotation2d.fromDegrees(-90.0)),
            Pose2d(24.0 + 120.0, 240.0 - 120.0, Rotation2d.fromDegrees(-180.0)),
            Pose2d(24.0, 240.0, Rotation2d.fromDegrees(-270.0))
        ),
        listOf<TimingConstraint<Pose2dWithCurvature>>(),
        12.0 * 12.0,
        12.0 * 12.0,
        9.0
    )
}

class TrajectoryViewCanvas(val ppi: Int, val fieldWidth: Double, val fieldHeight: Double, val trajectory: Trajectory<TimedState<Pose2dWithCurvature>>, val model: DifferentialDrivetrainModel): JPanel(true) {
    private fun horizontalInchesToPixels(inches: Double): Int {
        return width - (ppi * inches).roundToInt()
    }

    private fun verticalInchesToPixels(inches: Double): Int {
        return height - (ppi * inches).roundToInt()
    }

    data class TrajectoryStats(val maxVel: Double, val time: Double)

    private fun computeStats(): TrajectoryStats {
        var maxVel = 0.0

        for (i in 0 until trajectory.length()) {
            val state = trajectory.getState(i)
            val velocity = Twist2d(state.velocity(), 0.0, state.velocity() * state.state().curvature)
            val wheelVelocities = model.driveKinematicsModel.inverseKinematics(velocity)
            val leftVelocity = abs(wheelVelocities.left)
            val rightVelocity = abs(wheelVelocities.right)
            if (leftVelocity > maxVel) maxVel = leftVelocity
            if (rightVelocity > maxVel) maxVel = rightVelocity
        }

        val time = trajectory.lastState.t()

        return TrajectoryStats(maxVel, time)
    }

    private val executor = ScheduledThreadPoolExecutor(1)

    inner class Simulation(val rate: Double) {
        private val iterator = TrajectoryIterator(TimedView(trajectory))

        private fun timeSeconds(): Double {
            return System.nanoTime() * 1e-9 * rate
        }

        var startTime = 0.0
        var done = false

        var latestState = Pose2d.identity()
        var latestTime = 0.0

        fun start() {
            reset()
            startTime = timeSeconds()
            //Run at 60 fps
            executor.scheduleAtFixedRate(::update, 0L, ((1000.0 / 60.0) * 1e6).roundToLong(), TimeUnit.NANOSECONDS)
        }

        fun reset() {
            done = false
            executor.queue.clear()
            SwingUtilities.invokeLater {
                latestTime = 0.0
                latestState = trajectory.firstState.state().pose
                repaint()
            }
        }

        //Updates the simulation
        private fun update() {
            var time = timeSeconds() - startTime
            if (time > iterator.remainingProgress) {
                time = iterator.remainingProgress
                done = true
            }
            val state = iterator.preview(time).state()

            SwingUtilities.invokeLater {
                latestTime = time
                latestState = state.state().pose
                repaint()
            }

            if (done) throw RuntimeException() //This is a hack to easily stop the task on the executor
        }
    }

    private var activeSimulation = Simulation(1.0)

    private val stats = computeStats()
    private val pathStroke = BasicStroke(2.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL)
    private val leftTrackTransform = Pose2d(0.0, model.geometryConstants.wheelbase.value / 2.0, Rotation2d.identity())
    private val rightTrackTransform = Pose2d(0.0, model.geometryConstants.wheelbase.value / -2.0, Rotation2d.identity())
    private val frontTransform = Pose2d(model.geometryConstants.wheelbase.value / 2.0, 0.0, Rotation2d.identity())
    private val backTransform = Pose2d(model.geometryConstants.wheelbase.value / -2.0, 0.0, Rotation2d.identity())

    private fun drawRobot(g: Graphics2D) {
        val currentPose = activeSimulation.latestState
        val horiz = horizontalInchesToPixels(currentPose.translation.y())
        val vert = verticalInchesToPixels(currentPose.translation.x())

        //Draw the dot on the robot origin

        g.color = Color.BLACK
        g.fillOval(horiz - 3, vert - 3, 6, 6)

        //Draw the robot bounding box
        val robotFrontLeft = currentPose.transformBy(frontTransform).transformBy(leftTrackTransform)
        val robotFrontRight = currentPose.transformBy(frontTransform).transformBy(rightTrackTransform)
        val robotBackLeft = currentPose.transformBy(backTransform).transformBy(leftTrackTransform)
        val robotBackRight = currentPose.transformBy(backTransform).transformBy(rightTrackTransform)

        val frontLeftHoriz = horizontalInchesToPixels(robotFrontLeft.translation.y())
        val frontLeftVert = verticalInchesToPixels(robotFrontLeft.translation.x())
        val frontRightHoriz = horizontalInchesToPixels(robotFrontRight.translation.y())
        val frontRightVert = verticalInchesToPixels(robotFrontRight.translation.x())
        val backLeftHoriz = horizontalInchesToPixels(robotBackLeft.translation.y())
        val backLeftVert = verticalInchesToPixels(robotBackLeft.translation.x())
        val backRightHoriz = horizontalInchesToPixels(robotBackRight.translation.y())
        val backRightVert = verticalInchesToPixels(robotBackRight.translation.x())

        g.color = Color.YELLOW
        g.drawLine(frontLeftHoriz, frontLeftVert, frontRightHoriz, frontRightVert) //front left -> front right
        g.color = Color.BLACK
        g.drawLine(frontRightHoriz, frontRightVert, backRightHoriz, backRightVert) //front right -> back right
        g.drawLine(backRightHoriz, backRightVert, backLeftHoriz, backLeftVert) //back right -> back left
        g.drawLine(backLeftHoriz, backLeftVert, frontLeftHoriz, frontLeftVert) //back left -> front left
    }

    private val fmt = DecimalFormat("0.####")
    private val velGradient = GradientPaint(10f, 60f, Color.GREEN, 130f, 60f, Color.RED)

    private fun drawText(g: Graphics2D) {
        val time = fmt.format(activeSimulation.latestTime)
        val state = activeSimulation.latestState

        g.color = Color.BLACK
        g.drawString("time: $time s", 10, 20)
        g.drawString("pose: $state", 10, 40)

        g.paint = velGradient
        g.fillRect(10, 60, 120, 10)

        g.color = Color.BLACK
        g.drawString("${fmt.format(stats.maxVel)} in/s", 135, 70)
    }

    private fun drawTrajectory(g: Graphics2D) {
        g.stroke = pathStroke
        for (i in 1 until trajectory.length()) {
            val curState = trajectory.getState(i)
            val lastState = trajectory.getState(i - 1)

            val curStateLeft = curState.state().transformBy(leftTrackTransform)
            val lastStateLeft = lastState.state().transformBy(leftTrackTransform)
            val curStateRight = curState.state().transformBy(rightTrackTransform)
            val lastStateRight = lastState.state().transformBy(rightTrackTransform)

            val curHoriz = horizontalInchesToPixels(curState.state().translation.y())
            val curVert = verticalInchesToPixels(curState.state().translation.x())
            val lastHoriz = horizontalInchesToPixels(lastState.state().translation.y())
            val lastVert = verticalInchesToPixels(lastState.state().translation.x())

            val curLeftHoriz = horizontalInchesToPixels(curStateLeft.translation.y())
            val curLeftVert = verticalInchesToPixels(curStateLeft.translation.x())
            val lastLeftHoriz = horizontalInchesToPixels(lastStateLeft.translation.y())
            val lastLeftVert = verticalInchesToPixels(lastStateLeft.translation.x())

            val curRightHoriz = horizontalInchesToPixels(curStateRight.translation.y())
            val curRightVert = verticalInchesToPixels(curStateRight.translation.x())
            val lastRightHoriz = horizontalInchesToPixels(lastStateRight.translation.y())
            val lastRightVert = verticalInchesToPixels(lastStateRight.translation.x())

            val curVel = model.driveKinematicsModel.inverseKinematics(Twist2d(curState.velocity(), 0.0, curState.velocity() * curState.state().curvature))
            val leftWeight = abs(curVel.left) / stats.maxVel
            val rightWeight = abs(curVel.right) / stats.maxVel
            val leftColor = Color((255 * leftWeight).roundToInt(), (255 * (1 - leftWeight)).roundToInt(), 0)
            val rightColor = Color((255 * rightWeight).roundToInt(), (255 * (1 - rightWeight)).roundToInt(), 0)
            g.color = Color.BLUE
            g.drawLine(curHoriz, curVert, lastHoriz, lastVert)
            g.color = leftColor
            g.drawLine(curLeftHoriz, curLeftVert, lastLeftHoriz, lastLeftVert)
            g.color = rightColor
            g.drawLine(curRightHoriz, curRightVert, lastRightHoriz, lastRightVert)
        }
    }

    //Main rendering function
    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val g2d = g as Graphics2D
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        drawTrajectory(g2d)
        drawRobot(g2d)
        drawText(g2d)
    }

    fun startSimulation() {
        activeSimulation.start()
    }

    fun resetSimulation() {
        activeSimulation.reset()
    }

    init {
        preferredSize = Dimension(ceil(fieldWidth * ppi).toInt(), ceil(fieldHeight * ppi).toInt())
        border = BorderFactory.createLineBorder(Color.BLACK, 1)
        resetSimulation()
    }
}

class TrajectoryVisualizer(val trajectory: Trajectory<TimedState<Pose2dWithCurvature>>, val model: DifferentialDrivetrainModel) {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        val canvas = TrajectoryViewCanvas(2, 250.0, 250.0, trajectory, model)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.X_AXIS)

        val playButton = JButton("Simulate")
        playButton.addActionListener { canvas.startSimulation() }
        val resetButton = JButton("Reset")
        resetButton.addActionListener { canvas.resetSimulation() }

        frame.addWindowListener(object : WindowAdapter() {
            override fun windowClosing(e: WindowEvent?) {
                exitProcess(0) //Force the application to close
            }
        })

        buttonPanel.add(playButton)
        buttonPanel.add(resetButton)

        frame.layout = BoxLayout(frame.contentPane, BoxLayout.Y_AXIS)

        frame.contentPane.add(canvas)
        frame.contentPane.add(buttonPanel)

        SwingUtilities.invokeLater {
            frame.pack()
            frame.isVisible = true
        }
    }
}

fun main() {
    val sim = TrajectoryVisualizer(DriveTrajectories.testTrajectory, DrivetrainSubsystem.model)
    sim.start()
}