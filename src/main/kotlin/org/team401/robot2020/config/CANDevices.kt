package org.team401.robot2020.config

object CANDevices {
    data class PoweredCANDevice(val canID: Int, val pdpChannel: Int)

    //All drive motors are Talon FX
    val driveLeftFrontMotor = PoweredCANDevice(0, 0)
    val driveLeftRearMotor = PoweredCANDevice(1, 0)
    val driveRightFrontMotor = PoweredCANDevice(2, 0)
    val driveRightRearMotor = PoweredCANDevice(3, 0)

    //Pigeon IMU CAN ID
    const val pigeon = 0

    //Ball subsystem
    val turretRotationMotor = PoweredCANDevice(1, 0) //SPARK MAX + NEO 550
    val flyingVMotors = PoweredCANDevice(0, 0) //Victor SPX + 2 bag motors
    val towerMotor = PoweredCANDevice(2, 0) //SPARK MAX + NEO 550
    val intakeWheelsMotor = PoweredCANDevice(25, 0) //SPARK MAX + NEO 550
    val intakePivotMotor = PoweredCANDevice(4, 0) //SPARK MAX + NEO 550
    val kickerMotor = PoweredCANDevice(5, 0) //SPARK MAX + NEO 550
    val shooterMotorA = PoweredCANDevice(6, 0) //SPARK MAX + NEO
    val shooterMotorB = PoweredCANDevice(7, 0) //SPARK MAX + NEO
    val climbLeftElevatorMotor = PoweredCANDevice(8, 0) //SPARK MAX + NEO 550
    val climbRightElevatorMotor = PoweredCANDevice(9, 0) //SPARK MAX + NEO 550
    const val climbLeftElevatorEncoder = 0
    const val climbRightElevatorEncoder = 1

    //Spinner
    val spinnerMotor = PoweredCANDevice(10, 0) //SPARK MAX + NEO 550
}