// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class RunRollersUntilDetected extends RunRollersCommand {
  /** Creates a new RunRollersUntilDetected. */
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

  public RunRollersUntilDetected(double power) {
    super(power);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intakeSubsystem.gamePieceDetected() == true) {
      return true;
    }
    return false;
  }
}
