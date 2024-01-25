// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HardStopMotorSubsystem;

public class SetMotorPositionCommand extends Command {
  HardStopMotorSubsystem hardStopMotorSubsystem;
  double position;
  /** Creates a new SetMotorPositionCommand. */
  public SetMotorPositionCommand(HardStopMotorSubsystem _hardStopMotorSubsystem, double _position) {
    this.hardStopMotorSubsystem = _hardStopMotorSubsystem;
    this.position = _position;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.hardStopMotorSubsystem.setExtension(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
