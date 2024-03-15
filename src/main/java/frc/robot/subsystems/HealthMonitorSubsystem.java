package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkBase;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HealthMonitorSubsystem extends SubsystemBase {
  boolean healthy;

  public HealthMonitorSubsystem() {
  }

  @Override
  public void periodic() {
    List<String> broken = new ArrayList<>();

    // TODO this is for testing remove it!
    // broken.add("testing!!!");

    for (var nameAndAzimuthMotor : RobotContainer.swerveAzimuthMotors.entrySet()) {
      Object motor = nameAndAzimuthMotor.getValue();
      if (motor instanceof CANSparkBase) {
        CANSparkBase m = (CANSparkBase) motor;
        double t = m.getMotorTemperature();
        if (t > 50) {
          broken.add (nameAndAzimuthMotor.getKey() + " azimuth hot: " + Double.toString(t));
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
