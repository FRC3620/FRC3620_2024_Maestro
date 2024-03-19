package frc.robot.commands;

import java.util.function.DoubleSupplier;

public class RunIndexerUntilGamePieceDetectedCommand extends RunIndexerCommand {
  public RunIndexerUntilGamePieceDetectedCommand(DoubleSupplier powerSupplier) {
    super(powerSupplier);
  }
  
  @Override
  public boolean isFinished() {
    return gamePieceDetected();
  }
}
