package org.usfirst.frc3620.misc;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class CANSparkMaxSendable extends CANSparkMax implements Sendable  {
    public CANSparkMaxSendable(int deviceId, MotorType motorType) {
        super(deviceId, motorType);

        SendableRegistry.addLW(this, "SparkMax", deviceId);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}