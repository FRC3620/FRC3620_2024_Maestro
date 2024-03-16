package frc.robot.commands;

import org.usfirst.frc3620.misc.XBoxConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualMoveCommand extends Command {
  IntakeSubsystem subsystem = RobotContainer.intakeSubsystem;

  public IntakeManualMoveCommand() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double power = RobotContainer.operatorJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X);
    subsystem.setManualShoulderPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setManualShoulderPower(null);
  }
}
