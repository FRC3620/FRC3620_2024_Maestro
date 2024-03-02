// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class goInSixInchesCommand extends Command {
  RollersSubsystem rollers = RobotContainer.rollersSubsystem;
  double startPos = 0;
  /** Creates a new goInSixInchesCommand. */
  public goInSixInchesCommand() {

    addRequirements(rollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollers.getLimitSwitch().enableLimitSwitch(false);
    startPos = rollers.getRollerPosition();
    rollers.setRollerPower(0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("rollersStartPosition", startPos);
    SmartDashboard.putNumber("rollersGoalPosition", startPos + 2);
    SmartDashboard.putNumber("rollersPosition", rollers.getRollerPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollers.getLimitSwitch().enableLimitSwitch(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rollers.getRollerPosition() > startPos + 2 ? true : false;
  }
}
