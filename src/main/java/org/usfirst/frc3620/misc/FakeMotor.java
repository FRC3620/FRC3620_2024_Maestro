package org.usfirst.frc3620.misc;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Add your docs here. */
public class FakeMotor implements Sendable, MotorController, AutoCloseable {
    Logger logger = EventLogging.getLogger(this.getClass(), Level.INFO);
    int deviceId;
    double speed;
    boolean inverted;

    public FakeMotor(int deviceId) {
        this.deviceId = deviceId;
        speed = 0;
        inverted = false;
        SendableRegistry.addLW(this, "FakeMotor", deviceId);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    @Override
    public void set(double speed) {
        this.speed = speed;
        logger.info ("FakeMotor[{}].set({})", deviceId, speed);
    }

    @Override
    public double get() {
        return speed;
    }

    @Override
    public void setInverted(boolean isInverted) {
        this.inverted = isInverted;
        logger.info ("FakeMotor[{}].setInverted({})", deviceId, isInverted);
    }

    @Override
    public boolean getInverted() {
        return this.inverted;
    }

    @Override
    public void disable() {
        logger.info ("FakeMotor[{}].disable()", deviceId);
    }

    @Override
    public void stopMotor() {
        logger.info ("FakeMotor[{}].stopMotor()", deviceId);
        this.speed = 0;
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }

}
