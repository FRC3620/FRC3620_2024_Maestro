package org.usfirst.frc3620.misc;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Add your docs here. */
public class CANSparkMaxSendableWrapper implements Sendable {
    Logger logger = EventLogging.getLogger(this.getClass(), Level.INFO);
    CANSparkMax motor;
    int deviceId;

    public CANSparkMaxSendableWrapper(CANSparkMax motor) {
        this.motor = motor;
        deviceId = -1;
        if (motor != null)
            deviceId = motor.getDeviceId();
        SendableRegistry.addLW(this, "CANSparkMaxSendableWrapper", deviceId);
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
            logger.info("CANSparkMaxSendableWrapper[{}].set({})", deviceId, speed);
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
            logger.info("CANSparkMaxSendableWrapper[{}].stopMotor()", deviceId);
            if (motor != null)
                motor.stopMotor();
        } catch (Exception ex) {
            logger.error("boom: {}", ex);
        }
    }

    @Override
    public String toString() {
        return "CANSparkMaxSendableWrapper [deviceId=" + deviceId + ", motor=" + motor + "]";
    }

}
