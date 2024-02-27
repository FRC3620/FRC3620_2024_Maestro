package frc.robot.commands;

public class RunRollersUntilDetected extends RunRollersCommand {
  public RunRollersUntilDetected(Double power) {
    super(power);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.gamePieceDetected();
  }
}
