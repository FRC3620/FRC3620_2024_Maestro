// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbElevationSubsystem;

public class SetClimberPowerPositionCommand extends Command {
  ClimbElevationSubsystem climbElevationSubsystem;
  /** Creates a new SetClimberPowerPositionCommand. */

  public SetClimberPowerPositionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    climbElevationSubsystem = RobotContainer.climbElevationSubsystem;
    addRequirements(climbElevationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // checks if the left joystick is running up or down
    // if joystick position is above 0 (running up), the joystick will run on
    // positive power, and if its below 0, the joystick will run on negative power
    double power = -RobotContainer.getClimberJoystickPosition();
    if (Math.abs(RobotContainer.getClimberJoystickPosition()) < .2) {
      climbElevationSubsystem.setPower(0);
    } else {
      climbElevationSubsystem.setPower(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbElevationSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
