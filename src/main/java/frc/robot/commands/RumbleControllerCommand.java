// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Timer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleControllerCommand extends Command {
  /** Creates a new RumbleControllerCommand. */
  Joystick controller;
  double rumbleAmount;
  public RumbleControllerCommand(Joystick controller, double rumbleAmount) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.rumbleAmount = rumbleAmount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setRumble(RumbleType.kBothRumble, rumbleAmount);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    new Timer();
    return false;
  }
}
