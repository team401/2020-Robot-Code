package org.team401.robot2020.config

object CANDevices {
    data class PoweredCANDevice(val canID: Int, val pdpChannel: Int)

    //All drive motors are NEO motors powered by Spark Max Devices
    val driveLeftFrontMotor = PoweredCANDevice(15, 0)
    val driveLeftRearMotor = PoweredCANDevice(16, 0)
    val driveRightFrontMotor = PoweredCANDevice(17, 0)
    val driveRightRearMotor = PoweredCANDevice(18, 0)

    //Pigeon IMU CAN ID
    const val pigeon = 0

    //Ball subsystem
    val turretRotationMotor = PoweredCANDevice(8, 0) //SPARK MAX + NEO 550
    val flyingVMotorLeft = PoweredCANDevice(7, 0) //Victor SPX + 775 Pro
    val flyingVMotorRight = PoweredCANDevice(6, 0) //Victor SPX + 775 Pro
    val towerMotor = PoweredCANDevice(5, 0) //TalonFX + Falcon 500
    val intakeWheelsMotor = PoweredCANDevice(1, 0) //Victor SPX + 775 Pro
    val kickerMotor = PoweredCANDevice(10, 0) //SPARK MAX + NEO 550
    val shooterMotorA = PoweredCANDevice(11, 0) //TalonFX + Falcon 500
    val shooterMotorB = PoweredCANDevice(12, 0) //TalonFX + Falcon 500
    val climbLeftElevatorMotor = PoweredCANDevice(4, 0) //Talon SRX + 775 Pro
    val climbRightElevatorMotor = PoweredCANDevice(3, 0) //Talon SRX + 775 Pr

    //Spinner
    val spinnerMotor = PoweredCANDevice(9, 0) //SPARK MAX + NEO 550
}