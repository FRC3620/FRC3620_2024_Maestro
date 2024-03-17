package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
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

  public TalonFX motor;

  public IndexerSubsystem() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
    boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, MOTORID_INDEXER, "Indexer")
        || shouldMakeAllCANDevices) {
      motor = new TalonFX(MOTORID_INDEXER);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(40).setCoast(false);
      motorSetup.apply(motor);
      addChild("motor", motor);
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

  HardwareLimitSwitchConfigs limitSwitchEnabled, limitSwitchDisabled;
  {
    limitSwitchDisabled = new HardwareLimitSwitchConfigs().withForwardLimitEnable(false);
    limitSwitchEnabled = new HardwareLimitSwitchConfigs().withForwardLimitEnable(true);
  }

  public void setLimitSwitchEnabled (boolean enabled) {
    motor.getConfigurator().apply(enabled ? limitSwitchEnabled : limitSwitchDisabled);
  }

  public boolean gamePieceDetected() {
    if (motor != null ){
      var ss = motor.getForwardLimit();
      // TODO need to verify!
      return ss.getValue() == ForwardLimitValue.ClosedToGround;
    }
    return false;
  }

  public double getVelocity() {
    if (motor == null)
      return 0;
    return motor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void updateTelemetry() {
    SmartDashboard.putBoolean(name + ".game_piece_detected", gamePieceDetected());
    if (motor != null) {
      SmartDashboard.putNumber(name + ".motor_current", motor.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber(name + ".power", motor.getDutyCycle().getValueAsDouble() / 100.0);
      SmartDashboard.putNumber(name + ".temperature", motor.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber(name + ".speed", motor.getRotorVelocity().getValueAsDouble());
    }
  }
  
}
