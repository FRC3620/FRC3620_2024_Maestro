package frc.robot.blinky;

public class LightSegment {
  int first;
  int last;

  Pattern currentPattern;

  public LightSegment(int first, int last) {
    this.first = first;
    this.last = last;
  }

  /**
   * Define what pattern the light segment should display.
   * 
   * @param pattern
   */
  public void setPattern(Pattern pattern) {
    if (pattern == currentPattern) return;
    if (currentPattern != null) {
      currentPattern.done(this);
    }
    currentPattern = pattern;
    currentPattern.start(this);
  }

  public void periodic() {
    if (currentPattern != null) {
      currentPattern.periodic(this);
    }
  }

  public int getFirst() {
    return first;
  }

  public int getLast() {
    return last;
  }

}
