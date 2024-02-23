package frc.robot.commands;

public class RunRollersUntilGone extends RunRollersCommand {
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ! subsystem.gamePieceDetected();
  }
}
