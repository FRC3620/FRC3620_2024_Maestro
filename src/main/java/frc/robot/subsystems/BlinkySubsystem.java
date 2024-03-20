package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.blinky.Pattern;

/**/
public class BlinkySubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass());

  AddressableLED leds;
  AddressableLEDBuffer lBuffer;
  Timer timer;
  List<LightSegment> lightSegments = new ArrayList<>();
  public LightSegment lightSegment = new LightSegment(0, 19);

  /** Creates a new BlinkySubsystem. */
  public BlinkySubsystem() {
    leds = new AddressableLED(0);
    lBuffer = new AddressableLEDBuffer(20);
    timer = new Timer();
    leds.setLength(lBuffer.getLength());
    leds.start();
  }

  @Override
  public void periodic() {
    boolean changed = false;
    for (var lightSegment : lightSegments) {
      if (lightSegment.runPattern()) {
        changed = true;
      }
    }
    if (changed) {
      leds.setData(lBuffer);
    }
  }

  public LightSegment getLightSegment(int first, int last) {
    LightSegment rv = new LightSegment(first, last);
    lightSegments.add(rv);
    return rv;
  }

  public class LightSegment extends SubsystemBase {
    int segment_first;
    int segment_length;

    Pattern currentPattern;

    boolean patternChanged = false;

    public LightSegment(int first, int last) {
      this.segment_first = first;
      this.segment_length = (last - first) + 1;
      setName(getName() + "[" + first + "-" + last + "]");
    }

    /**
     * Define what pattern the light segment should display.
     * 
     * @param pattern
     */
    public void setPattern(Pattern pattern) {
      if (pattern == currentPattern)
        return;
      // logger.info ("{} pattern set to {}", getName(), pattern.toString());
      patternChanged = true;
      if (currentPattern != null) {
        currentPattern.done(this);
      }
      currentPattern = pattern;
      currentPattern.start(this);
    }

    boolean runPattern() {
      boolean rv = patternChanged;
      patternChanged = false;
      if (currentPattern != null) {
        if (currentPattern.periodic(this)) {
          rv = true;
        }
      }
      return rv;
    }

    public void updateLEDs(Color color) {
      updateLEDs(0, segment_length, color);
    }

    public void updateLEDs(int first, Color color) {
      updateLEDs(first, first, color);
    }

    public void updateLEDs(int first, int last, Color color) {
      for (int i = first; i < last; i++) {
        lBuffer.setLED(i + segment_first, color);
      }
    }

    public int getLength() {
      return segment_length;
    }
  }
}
