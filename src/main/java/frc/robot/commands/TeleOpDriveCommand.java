package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends Command {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  private DriveSubsystem driveSubsystem;

  double strafeX;
  double strafeY;
  double spinXDriver;
  double spinXOperator; 
  double spinX;
  double desiredHeading;
  double currentHeading;
  /**
   * Creates a new TeleOpDriveCommand.
   */
  public TeleOpDriveCommand(DriveSubsystem m_driveSubsystem) {
    this.driveSubsystem = m_driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // logger.info("Init tod");
    desiredHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
    driveSubsystem.setTargetHeading(desiredHeading);
    driveSubsystem.setDriveToCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    strafeX = RobotContainer.getDriveHorizontalJoystick();
    strafeY = RobotContainer.getDriveVerticalJoystick();
    spinX = driveSubsystem.getSpinPower();

    driveSubsystem.teleOpDrive(strafeX, strafeY, spinX);


    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.teleOpDrive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
