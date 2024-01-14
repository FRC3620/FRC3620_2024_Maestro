// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunShooterMotorsCommand extends Command {
  /** Creates a new RunPropellorFromJoystickCommand. */
  public RunShooterMotorsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("motor1Requested.power", 0);
    SmartDashboard.putNumber("motor2Requested.power", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power1 = SmartDashboard.getNumber("motor1Requested.power", 0);
    double power2 = SmartDashboard.getNumber("motor2Requested.power", 0);
    RobotContainer.shooterSubsystem.spinMotor1(power1);
    RobotContainer.shooterSubsystem.spinMotor2(power2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.spinMotor1(0);
    RobotContainer.shooterSubsystem.spinMotor2(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
