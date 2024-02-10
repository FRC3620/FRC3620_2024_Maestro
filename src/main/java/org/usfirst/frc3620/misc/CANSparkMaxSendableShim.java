package org.usfirst.frc3620.misc;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class CANSparkMaxSendableShim implements Sendable, MotorController, AutoCloseable  {
    static Logger logger = EventLogging.getLogger(CANSparkMaxSendableShim.class, Level.INFO);

    CANSparkMax motor;
    int id = -1;

    public CANSparkMaxSendableShim(CANSparkMax motor) {
        this.motor = motor;
        if (motor != null) {
            id = motor.getDeviceId();
        }
        logger.info("CANSparkMaxSendableShim[{}] created", id);
        SendableRegistry.addLW(this, "SparkMax", id);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    @Override
    public void set(double d) {
        logger.info ("CANSparkMaxSendableShim[{}].set({})", id, d);
        if (motor != null) motor.set(d);
    }

    @Override
    public double get() {
        if (motor == null) return 0;
        return motor.get();
    }

    @Override
    public void stopMotor() {
       logger.info ("CANSparkMaxSendableShim[{}].stopMotor()", id);
       if (motor != null) motor.stopMotor();
    }

    @Override
    public void setInverted(boolean isInverted) {
        if (motor != null) motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        if (motor != null) return motor.getInverted();
        return false;
    }

    @Override
    public void disable() {
        if (motor != null) motor.disable();
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
}