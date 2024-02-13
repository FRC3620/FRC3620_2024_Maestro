package org.usfirst.frc3620.misc;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class CSPMXSW implements Sendable, MotorController, AutoCloseable {
    Logger logger = EventLogging.getLogger(this.getClass(), Level.INFO);
    CANSparkMax motor;
    int deviceId;

    public CSPMXSW(CANSparkMax motor) {
        this.motor = motor;
        deviceId = -1;
        if (motor != null)
            deviceId = motor.getDeviceId();
        SendableRegistry.addLW(this, "CSPMXSW", deviceId);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    public void set(double speed) {
        try {
            logger.info("CSPMXSW[{}].set({})", deviceId, speed);
            if (motor != null)
                motor.set(speed);
        } catch (Exception ex) {
            logger.error("boom: {}", ex);
        }
    }

    public double get() {
        try {
            if (motor != null)
                return motor.get();
        } catch (Exception ex) {
            logger.error("boom: {}", ex);
        }
        return 0;
    }

    public void stopMotor() {
        try {
            logger.info("CSPMXSW[{}].stopMotor()", deviceId);
            if (motor != null)
                motor.stopMotor();
        } catch (Exception ex) {
            logger.error("boom: {}", ex);
        }
    }

    @Override
    public String toString() {
        String m = "(null)";
        if (motor != null) m = motor.toString();
        return "CSPMXSW [deviceId=" + deviceId + ", motor=" + m + "]";
    }

    @Override
    public void setInverted(boolean isInverted) {
        try {
            logger.info("CSPMXSW[{}].setInverted({})", deviceId, isInverted);
            if (motor != null)
                motor.setInverted(isInverted);
        } catch (Exception ex) {
            logger.error("boom: {}", ex);
        }
    }

    @Override
    public boolean getInverted() {
        try {
            if (motor != null)
                return motor.getInverted();
        } catch (Exception ex) {
            logger.error("boom: {}", ex);
        }
        return false;
    }

    @Override
    public void disable() {
        try {
            logger.info("CSPMXSW[{}].disable()", deviceId);
            if (motor != null)
                motor.disable();
        } catch (Exception ex) {
            logger.error("boom: {}", ex);
        }
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }

}
