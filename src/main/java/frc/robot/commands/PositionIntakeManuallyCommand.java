package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeLocation;
import frc.robot.subsystems.IntakeSubsystem;

public class PositionIntakeManuallyCommand extends Command {
  IntakeSubsystem subsystem = RobotContainer.intakeSubsystem;

  /** Creates a new SetIntakeLocationCommand. */
  public PositionIntakeManuallyCommand() {
    SmartDashboard.putNumber("intake.manual.shoulder", 0);
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulder = SmartDashboard.getNumber("intake.manual.shoulder", 0);
    IntakeLocation location = new IntakeLocation(shoulder);
    subsystem.setLocation(location);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
