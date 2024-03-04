package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class XModeCommand extends Command {
  SwerveSubsystem subsystem = RobotContainer.drivebase;
  public XModeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.lock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  /*
   * the robot does not drive correctly after X mode. We need to
   * figure that out.
   */
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
