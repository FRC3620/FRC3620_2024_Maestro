package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.RobotContainer;

public class RunIndexerUntilGamePieceGoneCommand extends RunIndexerCommand {

  public RunIndexerUntilGamePieceGoneCommand(DoubleSupplier powerSupplier) {
    super(powerSupplier);
  }

  @Override
  public void initialize() {
    super.initialize();
    setLimitSwitchEnabled(false);
    RobotContainer.visionSubsystem.takeSnapshot();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    setLimitSwitchEnabled(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ! gamePieceDetected();
  }
}
