package org.team401.robot2020.subsystems

import org.snakeskin.component.SmartGearbox
import org.snakeskin.component.SparkMaxOutputVoltageReadingMode
import org.snakeskin.component.impl.NullSparkMaxDevice
import org.snakeskin.dsl.*
import org.team401.robot2020.config.CANDevices

object ShooterSubsystem: Subsystem() {
    private val shooterMotorA = Hardware.createBrushlessSparkMax(
        CANDevices.shooterMotorA.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )
    private val shooterMotorB = Hardware.createBrushlessSparkMax(
        CANDevices.shooterMotorB.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )

    private val shooterGearbox = SmartGearbox(shooterMotorA, shooterMotorB, ratioToSensor = 1.0)

    private val kickerMotor = Hardware.createBrushlessSparkMax(
        CANDevices.kickerMotor.canID,
        SparkMaxOutputVoltageReadingMode.MultiplyVbusDevice,
        mockProducer = NullSparkMaxDevice.producer
    )
}