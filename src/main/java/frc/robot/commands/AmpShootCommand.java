// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShootCommand extends SequentialCommandGroup {
  ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
  ShooterSpeedAndAngle shooterSpeedAndAngle = new ShooterSpeedAndAngle(10, 50);
  ShooterSpeedAndAngle stopShooterSpeedAndAngle = new ShooterSpeedAndAngle(0, 50);

  /** Creates a new AmpShootCommand. */
  public AmpShootCommand() {
    addCommands(
        new InstantCommand(() -> shooterSubsystem.setSpeedAndAngle(shooterSpeedAndAngle)),
        new InstantCommand(() -> shooterSubsystem.setAmpBarPosition(7)),
        new WaitUntilCommand(() -> shooterSubsystem.getAmpBarPosition() > 6),

        // need to add something for Shooter to be at speed and in correct position
        //?

        // need to work the indexer
        //?

        new WaitCommand(0.2),
        new InstantCommand(() -> shooterSubsystem.setAmpBarPosition(0)),
        new InstantCommand(() -> shooterSubsystem.setSpeedAndAngle(stopShooterSpeedAndAngle)));
  }
}
