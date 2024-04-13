// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.usfirst.frc3620.misc.BlinkinColor;
import org.usfirst.frc3620.misc.Utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;


public class ShuttleLightsAimCommand extends Command {
  BlinkySubsystem blinkySubsystem;
  SwerveSubsystem swerveSubsystem;

  static Double SHOOTER_AIM_OFFSET = 3.0;

  public static Translation2d blueCornerPose = new Translation2d(0.0, 8.2);
  public static Translation2d redCornerPose = new Translation2d(16.465042, 8.2);

  static Optional<Alliance> color;

  
  /** Creates a new ShuttleLightAimCommand. */
  public ShuttleLightsAimCommand(BlinkySubsystem blinky, SwerveSubsystem swerve) {
    this.blinkySubsystem = blinky;
    this.swerveSubsystem = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(blinky);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = swerveSubsystem.getPose();
    double currentRotation = currentPose.getRotation().getDegrees();
    double camYawToCorner = 0;
    color = DriverStation.getAlliance();

    if (color.get() == Alliance.Blue) {
      camYawToCorner = Utilities.normalizeAngle(currentPose.getTranslation().minus(blueCornerPose).getAngle().getDegrees() + SHOOTER_AIM_OFFSET);
    } else {
      camYawToCorner = Utilities.normalizeAngle(currentPose.getTranslation().minus(redCornerPose).getAngle().getDegrees()+SHOOTER_AIM_OFFSET);
    }

    if (Math.abs(currentRotation-camYawToCorner)<3.5){
      if(color.get() == Alliance.Blue){
        //blue
        if(currentPose.getX()<10.5){
          blinkySubsystem.setColor(BlinkinColor.HOT_PINK);
        }else{
          blinkySubsystem.setColor(BlinkinColor.BLUE);
        }
      } else {
        //red
        if(currentPose.getX()>5.5){
          blinkySubsystem.setColor(BlinkinColor.HOT_PINK);
        }else{
          blinkySubsystem.setColor(BlinkinColor.BLUE);
        }
        
      }
    }
    SmartDashboard.putNumber("camYawToCorner", camYawToCorner);
    SmartDashboard.putNumber("errorToCorner", camYawToCorner-currentRotation);
    SmartDashboard.putData("ShuttleLightsAimCommand", new ShuttleLightsAimCommand(blinkySubsystem, swerveSubsystem));
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
