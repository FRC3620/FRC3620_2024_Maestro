package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAnglePIDZapper extends InstantCommand {
  ShooterSubsystem subsystem = RobotContainer.shooterSubsystem;
  SparkPIDController pid;

  /** Creates a new ShooterAnglePIDZapper. */
  public ShooterAnglePIDZapper() {
    addRequirements(subsystem);
    if (subsystem.elevationMotor != null) {
      pid = subsystem.elevationMotor.getPIDController();

      SmartDashboard.putNumber("test.shooter.angle.pid.actual.kP", pid.getP());
      SmartDashboard.putNumber("test.shooter.angle.pid.actual.kI", pid.getI());
      SmartDashboard.putNumber("test.shooter.angle.pid.requested.kP", pid.getP());
      SmartDashboard.putNumber("test.shooter.angle.pid.requested.kI", pid.getI());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (subsystem.elevationMotor != null) {
      double kP = SmartDashboard.getNumber("test.shooter.angle.pid.requested.kP", 0);
      double kI = SmartDashboard.getNumber("test.shooter.angle.pid.requested.kI", 0);
      pid.setP(kP);
      pid.setI(kI);
      SmartDashboard.putNumber("test.shooter.angle.pid.actual.kP", pid.getP());
      SmartDashboard.putNumber("test.shooter.angle.pid.actual.kI", pid.getI());
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
