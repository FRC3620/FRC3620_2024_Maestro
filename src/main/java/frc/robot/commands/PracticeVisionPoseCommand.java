// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

public class PracticeVisionPoseCommand extends Command {
  Logger logger = EventLogging.getLogger(getClass());
  VisionSubsystem vision = RobotContainer.visionSubsystem;

  Translation2d blueSP = new Translation2d(1.3473171105886368,5.5430292057165325);
  Translation2d redSP = new Translation2d(14.6526828894,5.5430292057165325);
  Pose2d blueSPPose = new Pose2d(blueSP, Rotation2d.fromDegrees(0));
  Pose2d redSPPose = new Pose2d(blueSP, Rotation2d.fromDegrees(180));
  /** Creates a new PracticeVisionPoseCommand. */
  public PracticeVisionPoseCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.isDisabled()){
      Pose2d pose = null;
      var limelight_results = vision.getLastLimelightResults();
      double targetsSeen = limelight_results.targetingResults.targets_Fiducials.length;
      if(targetsSeen>1){
        pose  = limelight_results.targetingResults.getBotPose2d_wpiBlue();
      } else {
        logger.info ("only see {} targers, guessing at location", targetsSeen);
      }

      if (pose == null) {
        // assumes SP position based on color
        var color = DriverStation.getAlliance();
        if (color.isPresent()) {
          if (color.get() == Alliance.Red) {
            RobotContainer.drivebase.resetOdometry(redSPPose);
          }else{
            RobotContainer.drivebase.resetOdometry(blueSPPose);
          }
        }
      }

      if (pose != null) {
        logger.info ("Setting pose to {}", pose);
        RobotContainer.drivebase.resetOdometry(pose);
      }
    } else {
      logger.info ("Won't reset when enabled");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
