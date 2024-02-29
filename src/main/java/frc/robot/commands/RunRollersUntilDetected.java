package frc.robot.commands;

public class RunRollersUntilDetected extends RunRollersCommand {
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.gamePieceDetected();
  }
}
