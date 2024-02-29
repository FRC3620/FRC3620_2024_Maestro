package frc.robot.commands;

public class RunRollersUntilGone extends RunRollersCommand {

  @Override
  public void initialize() {
    super.initialize();
    subsystem.getLimitSwitch().enableLimitSwitch(false);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    subsystem.getLimitSwitch().enableLimitSwitch(true);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ! subsystem.gamePieceDetected();
  }
}
