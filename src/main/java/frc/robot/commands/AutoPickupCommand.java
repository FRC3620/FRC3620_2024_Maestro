// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickupCommand extends ParallelCommandGroup {
  /** Creates a new AutoPickupCommand. */
  VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;
  ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
  IndexerSubsystem indexerSubsystem = RobotContainer.indexerSubsystem;
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
  public AutoPickupCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //pickup
    //pitch
    //

      new GroundPickupCommand(),
      new WaitUntilCommand(() -> indexerSubsystem.gamePieceDetected()),
      new RunIndexerUntilGamePieceGoneCommand(() -> 0.8)
      //new AutoShooterVisionAngleAdjustmentCommand(visionSubsystem, shooterSubsystem)
      );
  }
}
