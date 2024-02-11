// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Add your docs here. */
public class CANSparkMaxSendableWrapper implements Sendable, MotorController{
    Logger logger = EventLogging.getLogger(this.getClass(), Level.INFO);
    CANSparkMax motor;
    int deviceId;

    public CANSparkMaxSendableWrapper(CANSparkMax motor) {
        this.motor = motor;
        this.deviceId = -1;
        if (motor != null) this.deviceId = motor.getDeviceId();
        SendableRegistry.addLW(this, "CANSparkMaxSendableWrapper", deviceId);
    }

    @Override
    public void set(double speed) {
        logger.info ("CANSparkMaxSendableWrapper[{}].set({})", deviceId, speed);
        if (motor != null) motor.set(speed);
    }

    @Override
    public double get() {
        if (motor != null) return motor.get();
        return 0;
    }

    @Override
    public void setInverted(boolean isInverted) {
        logger.info ("CANSparkMaxSendableWrapper[{}].setInverted({})", deviceId, isInverted);
        if (motor != null) motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        if (motor != null) return motor.getInverted();
        return false;
    }

    @Override
    public void disable() {
        logger.info ("CANSparkMaxSendableWrapper[{}].disable()", deviceId);
        if (motor != null) motor.disable();
    }

    @Override
    public void stopMotor() {
        logger.info ("CANSparkMaxSendableWrapper[{}].stopMotor()", deviceId);
        if (motor != null) motor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    @Override
    public String toString() {
        return "CANSparkMaxSendableWrapper [deviceId=" + deviceId + ", motor=" + motor + "]";
    }

}
