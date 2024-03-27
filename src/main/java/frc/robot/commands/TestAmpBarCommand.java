// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;

public class TestAmpBarCommand extends Command {
  ShooterWheelsAndAmpBarSubsystem shooterSubsystem = RobotContainer.shooterWheelsAndAmpBarSubsystem;
  //ShooterSpeedAndAngle shooterSpeedAndAngle = new ShooterSpeedAndAngle(1500, 54);
  //ShooterSpeedAndAngle stopShooterSpeedAndAngle = new ShooterSpeedAndAngle(0, 54);

  /** Creates a new AmpShootCommand. */
  public TestAmpBarCommand() {
    addRequirements(shooterSubsystem);
    
  }
  @Override
  public  void initialize(){
    shooterSubsystem.setAmpBarPosition(ShooterWheelsAndAmpBarSubsystem.AmpBarPosition.UP);
  }

  @Override
  public void end(boolean interrupted) {
      shooterSubsystem.setAmpBarPosition(ShooterWheelsAndAmpBarSubsystem.AmpBarPosition.HOME);
  }
}
