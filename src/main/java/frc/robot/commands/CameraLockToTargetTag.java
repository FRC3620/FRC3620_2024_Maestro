// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperSwerveController;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CameraLockToTargetTag extends Command {
  private boolean finishedTurn = false;
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final SuperSwerveController controller;

  private double headingToTag;

  /**
   * Creates a new CameraLockToTargetTag.
   * 
   * @param swerve The subsystem used by this command.
   */
  public CameraLockToTargetTag(SwerveSubsystem swerve, VisionSubsystem vision, SuperSwerveController controller) {

    this.swerve = swerve;
    this.vision = vision;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    addRequirements(vision);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    headingToTag = vision.camYawToTag();

    double headingError = headingToTag-swerve.getHeading().getDegrees();
    // putting current robot heading and target heading
    SmartDashboard.putNumber("vision.currentHeading", swerve.getHeading().getDegrees());
    SmartDashboard.putNumber("vision.headingToApril", headingToTag);
    SmartDashboard.putNumber("vision.headingError", headingError);
    // if target heading is greater than -300, execute turn
    if (headingToTag > -300) {
      // returns if command is turning
      SmartDashboard.putString("vision.IsCommandTurning?", "yes");
      // turns by target heading
      
      controller.turnTo(swerve, swerve.getHeading().getDegrees() + headingToTag);
      // if heading is close enough, stop turning
      if (swerve.getHeading().getDegrees() < headingToTag + 1 && swerve.getHeading().getDegrees() > headingToTag - 1) {
        finishedTurn = true;
        SmartDashboard.putString("vision.IsCommandTurning?", "no");
      } else {
        // if command isn't turning, return no
        finishedTurn = false;
        
      }

    }else{
      SmartDashboard.putString("vision.IsCommandTurning?", "no");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedTurn;
  }
}