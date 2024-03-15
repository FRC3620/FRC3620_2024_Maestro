package frc.robot.blinky;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

/** Add your docs here. */
public class ChasePattern extends Pattern {
  private double seconds = 0.25;
  private Color color1, color2;
  private int size1, size2;
  private Timer timer;
  private boolean showingColor1;

  public ChasePattern() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  public ChasePattern setColors(Color color1, Color color2) {
    this.color1 = color1;
    this.color2 = color2;
    return this;
  }

  public ChasePattern setColor1(Color color) {
    this.color1 = color;
    return this;
  }

  public ChasePattern setColor2(Color color) {
    this.color2 = color;
    return this;
  }

  public ChasePattern setBlinkTime(double seconds) {
    this.seconds = seconds;
    return this;
  }

  public void start(LightSegment lightSegment) {
    showingColor1 = true;
    timer.reset();
    timer.start();
  }

  public boolean periodic(LightSegment lightSegment) {
    if (timer.hasElapsed(seconds)) {
      if (showingColor1) {
        fill(lightSegment, color2, color1);
      } else {
        fill(lightSegment, color1, color2);
      }
      timer.restart();
      showingColor1 = !showingColor1;
      return true;
    }
    return false;
  }

  void fill (LightSegment lightSegment, Color c1, Color c2) {
    //         lightSegment.updateLEDs(color1);

  }

  @Override
  public String toString() {
    return "ChasePattern[" + color1.toString() + "," + color2.toString() + "]";
  }

}
