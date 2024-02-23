// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RollersSubsystem;

public class RunRollersCommand extends Command {
  /** Creates a new RunRollersUntilDetected. */
  RollersSubsystem subsystem = RobotContainer.rollersSubsystem;

  Double savedPower = null;

  public RunRollersCommand() {
    this(null);
  }

  public RunRollersCommand(Double power) {
    savedPower = power;
    if (power == null) {
      SmartDashboard.putNumber("rollers.manual.input", 0);
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = 0;
    if (savedPower == null) {
      power = SmartDashboard.getNumber("rollers.manual.power", 0);
    } else {
      power = savedPower;
    }
    subsystem.setRollerPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
