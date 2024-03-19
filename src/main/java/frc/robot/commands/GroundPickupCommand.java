package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.IntakeLocation;

public class GroundPickupCommand extends ParallelDeadlineGroup {
  public GroundPickupCommand() {
    // when this command is done, we are all done
    super(new RunIndexerAndRollersUntilGamePieceDetectedCommand());
    // these command get interrupted when the first one is done
    addCommands(new SetIntakeLocationCommand(IntakeLocation.IntakeOut));
  }
}
