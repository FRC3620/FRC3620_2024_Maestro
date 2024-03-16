// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShootCommand extends Command {
  ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
  ShooterSpeedAndAngle shooterSpeedAndAngle = new ShooterSpeedAndAngle(10, 50);
  ShooterSpeedAndAngle stopShooterSpeedAndAngle = new ShooterSpeedAndAngle(0, 50);

  /** Creates a new AmpShootCommand. */
  public AmpShootCommand() {
    
  }
  @Override
  public  void initialize(){
    shooterSubsystem.setSpeedAndAngle(shooterSpeedAndAngle);
     shooterSubsystem.setAmpBarPosition(7);
   
    // need to add something for Shooter to be at speed and in correct position?
    // need to work the indexer?

  }
  @Override
  public void end(boolean interrupted) {
      shooterSubsystem.setAmpBarPosition(0);
      shooterSubsystem.setSpeedAndAngle(stopShooterSpeedAndAngle);
  }
}
