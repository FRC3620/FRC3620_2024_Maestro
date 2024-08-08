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
  public enum Health { OK, SICK, REALLY_SICK }
  Health health = Health.OK;

  Set<String> broken = new LinkedHashSet<>(); // lLinkedHashSet remembers insertion order

  public HealthMonitorSubsystem() {
  }

  @Override
  public void periodic() {
    broken.clear();
    boolean reallyBad = false;

    for (var nameAndMotor : RobotContainer.swerveAzimuthMotors.entrySet()) {
      SwerveMotor swerveMotor = nameAndMotor.getValue();
      var motor = swerveMotor.getMotor();
      if (motor instanceof CANSparkBase) {
        @SuppressWarnings({ "resource" })
        CANSparkBase m = (CANSparkBase) motor;
        double t = m.getMotorTemperature();
        if (t > 70) {
          broken.add (nameAndMotor.getKey() + " azimuth hot: " + Double.toString(t));
        }
        if (t > 100) {
          reallyBad = true;
        }
      }
    }

    for (var nameAndMotor : RobotContainer.swerveDriveMotors.entrySet()) {
      SwerveMotor swerveMotor = nameAndMotor.getValue();
      var motor = swerveMotor.getMotor();
      if (motor instanceof CANSparkBase) {
        @SuppressWarnings({ "resource" })
        CANSparkBase m = (CANSparkBase) motor;
        double t = m.getMotorTemperature();
        if (t > 85) {
          broken.add (nameAndMotor.getKey() + " drive hot: " + Double.toString(t));
        }
        if (t > 105) {
          reallyBad = true;
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
          reallyBad = true;
        }
      }

    }


    health = reallyBad ? Health.REALLY_SICK : (broken.size() != 0 ? Health.SICK : Health.OK);

    SmartDashboard.putBoolean("health.ok", health == Health.OK);
    SmartDashboard.putString("health.details", String.join("; ", broken));
  }

  public Health getHealth() {
    return health;
  }
}
