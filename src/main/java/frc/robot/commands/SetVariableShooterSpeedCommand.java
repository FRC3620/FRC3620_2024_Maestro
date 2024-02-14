// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetVariableShooterSpeedCommand extends Command {
  ShooterSubsystem shooterSubsystem;

  /** Creates a new setShooterSpeedCommand. */
  public SetVariableShooterSpeedCommand(ShooterSubsystem shooterSubsystemP) {
    SmartDashboard.putNumber("shooter.speed", 0);
    shooterSubsystem = shooterSubsystemP;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystemP);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("shooter.speed", 0);
    // this.isFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = SmartDashboard.getNumber("shooter.speed", 0);
    shooterSubsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
