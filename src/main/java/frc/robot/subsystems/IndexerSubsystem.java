package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IndexerSubsystem extends SubsystemBase implements HasTelemetry {
  public final static int MOTORID_INDEXER = 9;
  boolean disabledForDebugging = false;
  String name = "indexer";
  
  // ---------------------------------------------------------------------

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public CANSparkMaxSendable motor;
  RelativeEncoder encoder;
  SparkLimitSwitch limitSwitch;

  public IndexerSubsystem() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
    boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INDEXER, "Indexer")
        || shouldMakeAllCANDevices) {
      motor = new CANSparkMaxSendable(MOTORID_INDEXER, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(40).setCoast(false);
      motorSetup.apply(motor);
      addChild("motor", motor);

      encoder = motor.getEncoder();
      encoder.setPositionConversionFactor(1);
      encoder.setVelocityConversionFactor(1);

      limitSwitch = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    }
  }

  public void periodic() {
  }

  /**
   * Sets the cannon to extend the arm to 'length' inches. Increasing
   * length is a longer arm.
   * "Extend" motor.
   */

  public void setPower(double speed) {
    if (motor != null) {
      if (!disabledForDebugging) {
        motor.set(speed);
      }
    }
  }

  public void setLimitSwitchEnabled (boolean enabled) {
    limitSwitch.enableLimitSwitch(enabled);
  }

  public boolean gamePieceDetected() {
    if (motor != null ){
      return limitSwitch.isPressed();
    }
    return false;
  }

  public double getVelocity() {
    if (encoder == null)
      return 0;
    return encoder.getVelocity();
  }

  @Override
  public void updateTelemetry() {
    SmartDashboard.putBoolean(name + ".game_piece_detected", gamePieceDetected());
    if (motor != null) {
      SmartDashboard.putNumber(name + ".motor_current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());
      SmartDashboard.putNumber(name + ".speed", encoder.getVelocity());
    }
  }
  
}
