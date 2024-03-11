package frc.robot.blinky;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

/** Add your docs here. */
public class BlinkPattern extends Pattern {
  private double onSeconds = 0.25;
  private double offSeconds = 0.25;
  private Color color;
  private Timer timer;
  private boolean on;

  public BlinkPattern() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  public BlinkPattern setColor(Color color) {
    this.color = color;
    return this;
  }

  public BlinkPattern setOnSeconds(double seconds) {
    this.onSeconds = seconds;
    return this;
  }

  public BlinkPattern setOffSeconds(double seconds) {
    this.offSeconds = seconds;
    return this;
  }

  public BlinkPattern setBlink(double seconds) {
    this.onSeconds = seconds;
    this.offSeconds = seconds;
    return this;
  }

  public void start(LightSegment lightSegment) {
    on = true;
    timer.reset();
    timer.start();
  }

  public boolean periodic(LightSegment lightSegment) {
    if (on) {
      if (timer.hasElapsed(onSeconds)) {
        lightSegment.updateLEDs(Color.kBlack);
        timer.restart();
        on = false;
        return true;
      }
    } else {
      if (timer.hasElapsed(offSeconds)) {
        lightSegment.updateLEDs(color);
        timer.restart();
        on = true;
        return true;
      }
    }
    return false;
  }
}
