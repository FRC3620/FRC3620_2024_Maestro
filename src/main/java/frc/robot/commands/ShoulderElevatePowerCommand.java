// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShoulderElevatePowerCommand extends Command {
  /** Creates a new CannonExtendCommand. */
  IntakeSubsystem intakeSubsystem;
  double desiredSpeed;
  double setPoint;
  double startTime;

  public ShoulderElevatePowerCommand(IntakeSubsystem _subsystem, double _desiredSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(_subsystem);
    intakeSubsystem = _subsystem;
    desiredSpeed = _desiredSpeed;
  } 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Double s = intakeSubsystem.getRequestedShoulderPosition();
    if (s == null) {
      setPoint = 0;
    } else {
      setPoint = s;
    }
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    intakeSubsystem.setShoulderPosition(setPoint+desiredSpeed*(Timer.getFPGATimestamp()-startTime));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}