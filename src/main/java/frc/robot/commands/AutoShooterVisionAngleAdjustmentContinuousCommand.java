// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.usfirst.frc3620.misc.RobotMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.ShooterCalcutlaiter;
import frc.robot.subsystems.ShooterElevationSubsystem;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoShooterVisionAngleAdjustmentContinuousCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final ShooterElevationSubsystem shooterElevationSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  boolean isFinished = false;

  /** Creates a new ShooterVisionAngleAdjustmentCommand. */
  public AutoShooterVisionAngleAdjustmentContinuousCommand(VisionSubsystem visionSubsystem,
      ShooterElevationSubsystem shooterElevationSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.shooterElevationSubsystem = shooterElevationSubsystem;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(shooterElevationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    Double distance = visionSubsystem.getCamDistToSpeaker();
    if(distance == null){
      SmartDashboard.putString("shooterVision.doIHaveTarget", "no");
    }else{
      ShooterSpeedAndAngle angle= ShooterCalcutlaiter.CalculaiteAngleM(distance);
      double desiredElevationPosition = angle.getPosition();
      shooterElevationSubsystem.setElevationPosition(angle.getPosition());
      SmartDashboard.putNumber("shooterVision.Angle", angle.getPosition());
      SmartDashboard.putString("shooterVision.doIHaveTarget", "yes");
      SmartDashboard.putNumber("shooterVision.Distance", distance);
      SmartDashboard.putNumber("shooterVision.DistanceFt", Units.metersToFeet(distance));
      SmartDashboard.putNumber("shooterVision.Yaw", visionSubsystem.getCamYawToSpeaker());
      double actualElevationPosition = shooterElevationSubsystem.getActualElevationPosition();
      
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
          if (ally.get() == Alliance.Red && swerveSubsystem.getPose().getX() > 8.3) {
              shooterElevationSubsystem.setElevationPosition(angle.getPosition());
              new InstantCommand(() -> swerveSubsystem.setAreWeAiming(true));
          } else if (ally.get() == Alliance.Blue && swerveSubsystem.getPose().getX() < 8.3) {
              shooterElevationSubsystem.setElevationPosition(angle.getPosition());
              new InstantCommand(() -> swerveSubsystem.setAreWeAiming(true));
          } else {
              shooterElevationSubsystem.setElevationPosition(ShooterSpeedAndAngle.sourcePickup);
              new InstantCommand(() -> swerveSubsystem.setAreWeAiming(false));
          }
      }
      
      //SmartDashboard.putNumber("shooterVision.desiredPosition", desiredElevationPosition);
      //SmartDashboard.putNumber("shooterVision.actualPosition", actualElevationPosition);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
