// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSpeedAndAngle;

public class SetShooterSpeedAndAngleAndWaitCommand extends SetShooterSpeedAndAngleCommand {

  private double buffer = 2.5;

  /** Creates a new keepMotorVeiocityCommand. */
  public SetShooterSpeedAndAngleAndWaitCommand(ShooterSpeedAndAngle speedAndAngle) {
    super(speedAndAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double requestedSpeed = speedAndAngle.getSpeed();
    double topCurrentSpeed = shooterSubsystem.getTopMotorVelocity();
    double bottomCurrentSpeed = shooterSubsystem.getBottomMotorVelocity();

    if (Math.abs(topCurrentSpeed - requestedSpeed) < buffer
        && Math.abs(bottomCurrentSpeed - requestedSpeed) < buffer) {
      return true;
    } else {
      return false;
    }
  }
}
