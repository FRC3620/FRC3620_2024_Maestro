package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;

public class ShooterSpeedPIDZapper extends InstantCommand {
  ShooterWheelsAndAmpBarSubsystem subsystem = RobotContainer.shooterWheelsAndAmpBarSubsystem;
  TalonFX motor;
  String prefix;

  /** Creates a new ShooterAnglePIDZapper. */
  public ShooterSpeedPIDZapper(TalonFX motor, String prefix) {
    this.motor = motor;
    this.prefix = prefix;

    addRequirements(subsystem);
    if (motor != null) {

      Slot0Configs config = new Slot0Configs();
      StatusCode ss = motor.getConfigurator().refresh(config);
      SmartDashboard.putString(prefix + ".pid.refresh.status", ss.toString());

      SmartDashboard.putNumber(prefix + ".pid.actual.kP", config.kP);
      SmartDashboard.putNumber(prefix + ".pid.actual.kI", config.kI);
      SmartDashboard.putNumber(prefix + ".pid.actual.kV", config.kV);
      SmartDashboard.putNumber(prefix + ".pid.requested.kP", config.kP);
      SmartDashboard.putNumber(prefix + ".pid.requested.kI", config.kI);
      SmartDashboard.putNumber(prefix + ".pid.requested.kV", config.kV);
    }
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (motor != null) {
    Slot0Configs config = new Slot0Configs();
    motor.getConfigurator().refresh(config);
    config.kP = SmartDashboard.getNumber(prefix + ".pid.requested.kP", 0);
    config.kI = SmartDashboard.getNumber(prefix + ".pid.requested.kI", 0);
    config.kV = SmartDashboard.getNumber(prefix + ".pid.requested.kV", 0);
    motor.getConfigurator().apply(config);

    motor.getConfigurator().refresh(config);
    SmartDashboard.putNumber(prefix + ".pid.actual.kP", config.kP);
    SmartDashboard.putNumber(prefix + ".pid.actual.kI", config.kI);
    SmartDashboard.putNumber(prefix + ".pid.actual.kV", config.kV);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
