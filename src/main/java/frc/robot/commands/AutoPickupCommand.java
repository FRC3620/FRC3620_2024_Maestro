// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;

public class AutoPickupCommand extends SequentialCommandGroup {
  /** Creates a new AutoPickupCommand. */
  IndexerSubsystem indexerSubsystem = RobotContainer.indexerSubsystem;

  public AutoPickupCommand() {
    addCommands(
      new WaitUntilCommand(() -> indexerSubsystem.gamePieceDetected()),
      new RunIndexerUntilGamePieceGoneCommand(() -> 0.8)
      );
  }
}
