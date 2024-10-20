// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class EnabledResetPoseWithVision extends Command {
  boolean poseFound = false;
  Logger logger = EventLogging.getLogger(getClass());

  /** Creates a new EnabledResetPoseWithVision. */
  public EnabledResetPoseWithVision() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    poseFound = false;
    //log initialization for visual update
    SmartDashboard.putString("Vision Pose Reset Status", "Initialized");
    logger.info("Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var lastResults = RobotContainer.visionSubsystem.getLastLimelightResults();
    int targetsSeen = lastResults.targets_Fiducials.length;
    Pose2d visPose = lastResults.getBotPose2d_wpiBlue();
    Pose2d vPose = visPose; // Our pose is set in VisionSubsystem periodic automatically when we see at least two tags
    double xmps = RobotContainer.drivebase.getRobotVelocity().vxMetersPerSecond;
    double ymps = RobotContainer.drivebase.getRobotVelocity().vyMetersPerSecond;
    double mps = Math.sqrt((xmps * xmps) + (ymps * ymps)); //calculates our meters per second
    if (mps < 2) {
      if (targetsSeen > 1) {
        //If we see 2 targets, and we are going slower than 2 mps, update odometry
        RobotContainer.drivebase.resetOdometry(vPose);
        //log success
        logger.info("Success! Updated Odometry to {}", vPose);
        SmartDashboard.putString("Vision Pose Reset Status", "Success! Updated Odometry to "+ vPose);
        poseFound = true;
      } else {
        //If we don't see two tags, log it
        logger.info("Waiting for 2 tags! I only see {}", targetsSeen);
        SmartDashboard.putString("Vision Pose Reset Status", "Waiting for 2 tags! I only see " + targetsSeen);
      }
    } else {
      //if we are going too fast, log it
      logger.info("Waiting to slow down! You are going {} mps", mps);
      SmartDashboard.putString("Vision Pose Reset Status", "Waiting to slow down! You are going "+ mps+" mps");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(poseFound){
      return true;
    }else{
      return false;
    }
  }
}
