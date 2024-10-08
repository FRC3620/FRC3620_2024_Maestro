// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterElevationSubsystem;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;

public class AmpShootCommand extends Command {
  ShooterElevationSubsystem shooterElevationSubsystem = RobotContainer.shooterElevationSubsystem;
  ShooterWheelsAndAmpBarSubsystem shooterWheelsAndAmpBarSubsystem = RobotContainer.shooterWheelsAndAmpBarSubsystem;
  ShooterSpeedAndAngle shooterSpeedAndAngle = new ShooterSpeedAndAngle(1100, 54);
  ShooterSpeedAndAngle stopShooterSpeedAndAngle = new ShooterSpeedAndAngle(0, 54);

  /** Creates a new AmpShootCommand. */
  public AmpShootCommand() {
    addRequirements(shooterElevationSubsystem, shooterWheelsAndAmpBarSubsystem);
  }

  @Override
  public void initialize(){
    shooterElevationSubsystem.setElevationPosition(shooterSpeedAndAngle);
    shooterWheelsAndAmpBarSubsystem.setRequestedWheelSpeed(shooterSpeedAndAngle);

    shooterWheelsAndAmpBarSubsystem.setAmpBarPosition(ShooterWheelsAndAmpBarSubsystem.AmpBarPosition.UP);
  }

  @Override
  public void end(boolean interrupted) {
      shooterWheelsAndAmpBarSubsystem.setAmpBarPosition(ShooterWheelsAndAmpBarSubsystem.AmpBarPosition.HOME);
      shooterWheelsAndAmpBarSubsystem.setRequestedWheelSpeed(stopShooterSpeedAndAngle);
      shooterElevationSubsystem.setElevationPosition(stopShooterSpeedAndAngle);
      // leave the shooter where it is
  }
}
