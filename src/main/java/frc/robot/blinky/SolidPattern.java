package frc.robot.blinky;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

public class SolidPattern extends Pattern {
  private Color color;

  public SolidPattern setColor(Color color) {
    this.color = color;
    return this;
  }

  @Override
  public void start(LightSegment lightSegment) {
    lightSegment.updateLEDs(color);
  }

}
