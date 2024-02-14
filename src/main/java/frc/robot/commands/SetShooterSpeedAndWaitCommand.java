// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterSpeedAndWaitCommand extends SetShooterSpeedCommand {
  
  private double buffer = 2.5;

  /** Creates a new keepMotorVeiocityCommand. */
  public SetShooterSpeedAndWaitCommand(ShooterSubsystem _ShooterSubsystem, double _rSpeed) {
    super(_rSpeed, _ShooterSubsystem);
    }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooterSubsystem.getMotorVelocity(shooterSubsystem.topMotor) < speed + buffer
        && shooterSubsystem.getMotorVelocity(shooterSubsystem.topMotor) >speed - buffer
        && shooterSubsystem.getMotorVelocity(shooterSubsystem.bottomMotor) < speed + buffer
        && shooterSubsystem.getMotorVelocity(shooterSubsystem.bottomMotor) > speed - buffer) {
          return true;
    }else{
    return false;
    }
  }
}
