package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeLocation;

public class GroundToHomeCommand extends SequentialCommandGroup {
  public GroundToHomeCommand() {
    addCommands(
      new SetIntakeLocationCommand(IntakeLocation.IntakeIn)
    );
  }
}
