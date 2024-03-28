package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.CANSparkBase;

import java.util.LinkedHashSet;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.motors.SwerveMotor;

public class HealthMonitorSubsystem extends SubsystemBase {
  boolean healthy;
  Set<String> broken = new LinkedHashSet<>(); // lLinkedHashSet remembers insertion order

  public HealthMonitorSubsystem() {
  }

  @Override
  public void periodic() {
    broken.clear();

    // TODO this is for testing remove it!
    // broken.add("testing!!!");

    for (var nameAndAzimuthMotor : RobotContainer.swerveAzimuthMotors.entrySet()) {
      SwerveMotor swerveMotor = nameAndAzimuthMotor.getValue();
      var motor = swerveMotor.getMotor();
      if (motor instanceof CANSparkBase) {
        @SuppressWarnings({ "resource" })
        CANSparkBase m = (CANSparkBase) motor;
        double t = m.getMotorTemperature();
        if (t > 50) {
          broken.add (nameAndAzimuthMotor.getKey() + " azimuth hot: " + Double.toString(t));
        }
      }
    }

    for (var nameAndSwerveAbsoluteEncoder: RobotContainer.swerveAbsoluteEncoders.entrySet()) {
      SwerveAbsoluteEncoder swerveAbsoluteEncoder = nameAndSwerveAbsoluteEncoder.getValue();
      var absoluteEncoder = swerveAbsoluteEncoder.getAbsoluteEncoder();
      if (absoluteEncoder instanceof DutyCycleEncoder) {
        @SuppressWarnings({ "resource" })
        DutyCycleEncoder dutyCycleEncoder = (DutyCycleEncoder) absoluteEncoder;
        if (! dutyCycleEncoder.isConnected()) {
          broken.add(nameAndSwerveAbsoluteEncoder.getKey() + " absolute encoder disconnected");
        }
      }

    }
    boolean newHealthy = broken.size() == 0;

    healthy = newHealthy;

    SmartDashboard.putBoolean("health.ok", healthy);
    SmartDashboard.putString("health.details", String.join("; ", broken));
  }

  public boolean isHealthy() {
    return healthy;
  }
}
