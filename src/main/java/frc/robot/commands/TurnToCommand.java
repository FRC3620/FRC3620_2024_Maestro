// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperSwerveController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class TurnToCommand extends Command {
  /** Creates a new TurnToCommand. */
  private boolean finishedTurn = false;

  private final SwerveSubsystem swerve;
  private final SuperSwerveController controller;
  private final double targetHeading;

  public TurnToCommand(SwerveSubsystem swerve, SuperSwerveController controller, double targetHeading) {
    this.swerve = swerve;
    this.controller = controller;
    this.targetHeading = targetHeading;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finishedTurn = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.turnTo(swerve, targetHeading);

    if(swerve.getPose().getRotation().getDegrees() < targetHeading + 1.5 && swerve.getPose().getRotation().getDegrees() > targetHeading - 1.5) {
      finishedTurn = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedTurn;
  }
}
