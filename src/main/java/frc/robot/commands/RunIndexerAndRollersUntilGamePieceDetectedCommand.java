package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class RunIndexerAndRollersUntilGamePieceDetectedCommand extends ParallelDeadlineGroup {
  static double defaultIndexerPower = 0.7;
  static double defaultRollerPower = 1.0;
  public RunIndexerAndRollersUntilGamePieceDetectedCommand () {
    this (() -> defaultIndexerPower, () -> defaultRollerPower);

  }
  public RunIndexerAndRollersUntilGamePieceDetectedCommand(DoubleSupplier indexerPowerSupplier, DoubleSupplier rollerPowerSupplier) {
    // Add the deadline command in the super() call. Add other commands using addCommands().
    super(
      // this one sets the pace
      new RunIndexerUntilGamePieceDetectedCommand(indexerPowerSupplier),
      // these get interrupted when the first one is done
      new RunRollersCommand(rollerPowerSupplier)
    );
  }
}
