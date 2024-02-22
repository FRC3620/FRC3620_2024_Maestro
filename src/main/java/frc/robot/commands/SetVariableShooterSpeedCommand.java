package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterSubsystem;

public class SetVariableShooterSpeedCommand extends Command {
  ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;

  /** Creates a new setShooterSpeedCommand. */
  public SetVariableShooterSpeedCommand() {
    SmartDashboard.putNumber("shooter.manual.speed", 0);
    SmartDashboard.putNumber("shooter.manual.angle", 90);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = SmartDashboard.getNumber("shooter.manual.speed", 0);
    double position= SmartDashboard.getNumber("shooter.manual.angle", 90);
    ShooterSpeedAndAngle shooterSpeedAndAngle= new ShooterSpeedAndAngle(speed, position);
    shooterSubsystem.setSpeedAndAngle(shooterSpeedAndAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
