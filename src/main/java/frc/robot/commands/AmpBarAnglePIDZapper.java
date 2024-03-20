package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpBarAnglePIDZapper extends InstantCommand {
  ShooterSubsystem subsystem = RobotContainer.shooterSubsystem;
  SparkPIDController pid;

  /** Creates a new ShooterAnglePIDZapper. */
  public AmpBarAnglePIDZapper() {
    addRequirements(subsystem);
    if (subsystem.ampBarMotor != null) {

      pid = subsystem.ampBarPID;

      SmartDashboard.putNumber("test.ampbar.angle.pid.actual.kP", pid.getP());
      SmartDashboard.putNumber("test.ampbar.angle.pid.actual.kI", pid.getI());
      SmartDashboard.putNumber("test.ampbar.angle.pid.requested.kP", pid.getP());
      SmartDashboard.putNumber("test.ampbar.angle.pid.requested.kI", pid.getI());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (subsystem.ampBarMotor != null) {
      double kP = SmartDashboard.getNumber("test.ampbar.angle.pid.requested.kP", 0);
      double kI = SmartDashboard.getNumber("test.ampbar.angle.pid.requested.kI", 0);
      pid.setP(kP);
      pid.setI(kI);
      SmartDashboard.putNumber("test.ampbar.angle.pid.actual.kP", pid.getP());
      SmartDashboard.putNumber("test.ampbar.angle.pid.actual.kI", pid.getI());
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
