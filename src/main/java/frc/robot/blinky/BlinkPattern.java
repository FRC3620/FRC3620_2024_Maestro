package frc.robot.blinky;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

/** Add your docs here. */
public class BlinkPattern extends Pattern {
  private double color1Seconds = 0.25;
  private double color2Seconds = 0.25;
  private Color color1, color2;
  private Timer timer;
  private boolean showingColor1;

  public BlinkPattern() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  public BlinkPattern setColor(Color color) {
    this.color1 = color;
    this.color2 = Color.kBlack;
    return this;
  }

  public BlinkPattern setColor1(Color color) {
    this.color1 = color;
    return this;
  }

  public BlinkPattern setColor2(Color color) {
    this.color2 = color;
    return this;
  }

  public BlinkPattern setBlinkTime(double seconds) {
    this.color1Seconds = seconds;
    this.color2Seconds = seconds;
    return this;
  }

  public BlinkPattern setOnSeconds(double seconds) {
    this.color1Seconds = seconds;
    return this;
  }

  public BlinkPattern setOffSeconds(double seconds) {
    this.color2Seconds = seconds;
    return this;
  }

  public BlinkPattern setColor1Seconds(double seconds) {
    this.color1Seconds = seconds;
    return this;
  }

  public BlinkPattern setColor2Seconds(double seconds) {
    this.color2Seconds = seconds;
    return this;
  }

  public void start(LightSegment lightSegment) {
    showingColor1 = true;
    timer.reset();
    timer.start();
  }

  public boolean periodic(LightSegment lightSegment) {
    if (showingColor1) {
      if (timer.hasElapsed(color1Seconds)) {
        lightSegment.updateLEDs(color2);
        timer.restart();
        showingColor1 = false;
        return true;
      }
    } else {
      if (timer.hasElapsed(color2Seconds)) {
        lightSegment.updateLEDs(color1);
        timer.restart();
        showingColor1 = true;
        return true;
      }
    }
    return false;
  }

  @Override
  public String toString() {
    return "BlinkPattern[" + color1.toString() + "," + color2.toString() + "]";
  }

}
