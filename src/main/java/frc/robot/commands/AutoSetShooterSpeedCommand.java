// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;

public class AutoSetShooterSpeedCommand extends Command {
  ShooterWheelsAndAmpBarSubsystem shooterWheelsAndAmpBarSubsystem = RobotContainer.shooterWheelsAndAmpBarSubsystem;
  double speed;
  /** Creates a new SetShooterSpeedCommand. */
  public AutoSetShooterSpeedCommand(double speed) {
    this.speed = speed;
    addRequirements(shooterWheelsAndAmpBarSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWheelsAndAmpBarSubsystem.setRequestedWheelSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooterWheelsAndAmpBarSubsystem.setRequestedWheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
