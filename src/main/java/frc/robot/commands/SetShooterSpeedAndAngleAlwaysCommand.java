// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterElevationSubsystem;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;

public class SetShooterSpeedAndAngleAlwaysCommand extends Command {
  ShooterWheelsAndAmpBarSubsystem shooterWheelAndAmpBarSubsystem = RobotContainer.shooterWheelsAndAmpBarSubsystem;
  ShooterElevationSubsystem shooterElevationSubsystem = RobotContainer.shooterElevationSubsystem;

  ShooterSpeedAndAngle speedAndAngle;

  /** Creates a new setShooterSpeedCommand. */
  public SetShooterSpeedAndAngleAlwaysCommand(ShooterSpeedAndAngle speedAndAngle) {
    this.speedAndAngle = speedAndAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterWheelAndAmpBarSubsystem, shooterElevationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWheelAndAmpBarSubsystem.setRequestedWheelSpeed(speedAndAngle);
    shooterElevationSubsystem.setElevationPosition(speedAndAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
