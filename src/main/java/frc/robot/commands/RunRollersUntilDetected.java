package frc.robot.commands;

public class RunRollersUntilDetected extends RunRollersCommand {
  // Returns true when the command should end.
  public RunRollersUntilDetected() {
    super();
  }

  public RunRollersUntilDetected(double power) {
    super(power);
  }

  @Override
  public boolean isFinished() {
    return subsystem.gamePieceDetected();
  }
}
