package frc.robot.commands;

public class RunRollersUntilGone extends RunRollersCommand {

  public RunRollersUntilGone() {
    super();
  }

  public RunRollersUntilGone(double power) {
    super(power);
  }

  @Override
  public void initialize() {
    super.initialize();
    setLimitSwitchEnabled(false);
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
