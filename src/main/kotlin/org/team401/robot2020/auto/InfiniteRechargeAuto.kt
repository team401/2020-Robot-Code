package org.team401.robot2020.auto

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.*
import org.snakeskin.measure.Seconds
import org.team401.robot2020.auto.steps.DriveTrajectoryStep
import org.team401.robot2020.auto.steps.OpenLoopDriveStep
import org.team401.robot2020.auto.steps.WaitForOdometry
import org.team401.robot2020.control.drivetrain.DriveTrajectories
import org.team401.robot2020.control.robot.RobotState
import org.team401.robot2020.control.robot.SuperstructureManager
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

object InfiniteRechargeAuto: RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return auto {
            //Gen trajectory
            step {
                DriveTrajectories.Trench6Trajectories.generateTrench6TrajectoryCollect(SmartDashboard.getNumber("Wall to Left Bumper Inches", 100.0))
                RobotState.observeFieldToTarget(readTimestamp(), Pose2d(0.0, 226.533, Rotation2d.identity()))
            }

            //Drive and collect
            parallel {
                DriveTrajectoryStep(DriveTrajectories.Trench6Trajectories.trench6TrajectoryCollect, true)

                sequential {
                    step { SuperstructureManager.lockNearShot() }
                    delay(0.25.Seconds)
                    step { SuperstructureManager.startFiring() }
                }

                sequential {
                    step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 170.0))
                    step {
                        SuperstructureManager.unwindShooter()
                        SuperstructureManager.startIntaking()
                    }
                }
            }

            //Drive back
            parallel {
                DriveTrajectoryStep(DriveTrajectories.Trench6Trajectories.trench6TrajectoryReturn, false)
                step {
                    SuperstructureManager.stopIntaking()
                }
            }

            //Shoot
            sequential {
                step { SuperstructureManager.lockNearShot() }
                delay(0.25.Seconds)
                step { SuperstructureManager.startFiring() }
            }

            //Stop firing
            delay(6.0.Seconds)
            step { SuperstructureManager.unwindShooter() }
        }
    }
}