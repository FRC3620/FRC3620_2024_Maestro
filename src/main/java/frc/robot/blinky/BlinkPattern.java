package frc.robot.blinky;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class BlinkPattern extends Pattern {
  // private int onSec;
  private double Sec;
  private Color color;
  private Timer timer;
  private boolean lSate;

  public BlinkPattern() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  public BlinkPattern setColor(Color color) {
    this.color = color;
    return this;
  }

  public BlinkPattern setBlink(double Sec) {
    this.Sec = Sec;
    // this.onSec=onSec;
    return this;
  }

  public void start(LightSegment lightSegment) {
  }

  public void periodic(LightSegment lightSegment) {
    if (timer.hasElapsed(Sec)) {
      if (!lSate) {
        lSate = true;
        RobotContainer.blinkySubsystem.updateLEDS(lightSegment.first, lightSegment.last, Color.kBlack);
        timer.restart();
      } else if (lSate) {
        lSate = false;
        RobotContainer.blinkySubsystem.updateLEDS(lightSegment.first, lightSegment.last, color);
        timer.restart();
      }
    }
  }
}
