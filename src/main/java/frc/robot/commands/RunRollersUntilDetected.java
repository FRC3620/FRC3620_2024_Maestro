package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.blinky.LightSegment;
import frc.robot.blinky.SolidPattern;

public class RunRollersUntilDetected extends RunRollersCommand {
  // Returns true when the command should end.
  LightSegment lightSegment= new LightSegment(0, 19);
  public RunRollersUntilDetected() {
    super();
  }

  public RunRollersUntilDetected(double power) {
    super(power);
  }

  @Override
  public boolean isFinished() {
    RobotContainer.blinkySubsystem.lightSegment.setPattern(new SolidPattern().setColor(Color.kGreen));
     return subsystem.gamePieceDetected();
    
  }
}
