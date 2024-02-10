// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperSwerveController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

public class HatePathplannerCommand extends Command {
  /** Creates a new SuperSwereDrive. */
  // Use addRequirements() here to declare subsystem dependencies.
  private final SwerveSubsystem swerve;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public HatePathplannerCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    chassisSpeeds.vxMetersPerSecond = 0.15;
    chassisSpeeds.vyMetersPerSecond = 0.0;
    chassisSpeeds.omegaRadiansPerSecond = 0.2;
    swerve.setChassisSpeeds(chassisSpeeds);
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
