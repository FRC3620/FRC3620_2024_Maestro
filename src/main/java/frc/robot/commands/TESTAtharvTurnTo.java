// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.usfirst.frc3620.misc.Utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.SuperSwerveController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import frc.robot.subsystems.VisionSubsystem;


public class TESTAtharvTurnTo extends Command {
  /** Creates a new TESTAtharvTurnTo. */
  SwerveSubsystem swerve = RobotContainer.drivebase;
  VisionSubsystem vision = RobotContainer.visionSubsystem;
  private SuperSwerveController controller;

  //speaker positions
  public static Translation2d blueSpeakerPos = new Translation2d(0.076, 5.547868);
  public static Translation2d redSpeakerPos = new Translation2d(16.465042, 5.547868);

  public TESTAtharvTurnTo() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var color = DriverStation.getAlliance();
    var limelight_results = vision.getLastLimelightResults();
    double targetsSeen = limelight_results.targetingResults.targets_Fiducials.length;

    if(targetsSeen>1){
      Pose2d vPose = limelight_results.targetingResults.getBotPose2d_wpiBlue();
      Translation2d vTranslation = new Translation2d(vPose.getX(),vPose.getY());
      Rotation2d rotationNeeded = null;
        if(color.isPresent()){
          if(color.get()==Alliance.Blue){
            rotationNeeded = (vTranslation.minus(blueSpeakerPos).getAngle());
          }else{
            rotationNeeded = (vTranslation.minus(redSpeakerPos).getAngle());
          }
          double degNeeded = rotationNeeded.getDegrees();
          SmartDashboard.putNumber("Rotation Needed", degNeeded);
        }
      double degNeeded = rotationNeeded.getDegrees();
      controller.turnTo(swerve, degNeeded);
    }

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
