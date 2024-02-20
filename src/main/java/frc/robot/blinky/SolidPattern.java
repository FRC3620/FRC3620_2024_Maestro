package frc.robot.blinky;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;

public class SolidPattern extends Pattern {
  public LightSegment lightSegment = new LightSegment(0, 0);
  private Color color;

  public SolidPattern setColor(Color color) {
    this.color = color;
    return this;
  }

  @Override
  public void start(LightSegment lightSegment) {
  }

  @Override
  public void periodic(LightSegment lightSegment) {
    RobotContainer.blinkySubsystem.updateLEDS(lightSegment.first, lightSegment.last, color);
  }
}
