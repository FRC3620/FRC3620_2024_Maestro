// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class RunRollersCommand extends Command {
  RollersSubsystem rollersSubsystem = RobotContainer.rollersSubsystem;

  Logger logger = EventLogging.getLogger(getClass());

  DoubleSupplier savedPowerSupplier;

  public RunRollersCommand(DoubleSupplier powerSupplier) {
    savedPowerSupplier = requireNonNullParam(powerSupplier, "powerSupplier", "<init>");
    addRequirements(rollersSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = savedPowerSupplier.getAsDouble();
    rollersSubsystem.setRollerPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollersSubsystem.setRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
