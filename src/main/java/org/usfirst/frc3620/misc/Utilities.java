// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

import java.util.LinkedList;
import java.util.function.Consumer;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.SendableRegistry.CallbackData;

/** Add your docs here. */
public class Utilities {
  static Logger logger = EventLogging.getLogger(Utilities.class, Level.INFO);

  /**
   * This method makes sure the angle difference calculated falls between -180
   * degrees and 180 degrees
   * 
   * @param angle
   * @return
   */
  public static double normalizeAngle(double angle) {
    angle = angle % 360;

    if (angle > 180) {
      angle = -360 + angle;
    }
    if (angle <= -180) {
      angle = 360 + angle;
    }

    if (angle == -0) {
      angle = 0;
    }

    return angle;
  }

  public static void dumpSendables(String label, String subsystemName) {
    logger.info("Dumping Sendables: {}", label);
    SendableRegistry.foreachLiveWindow(0, new Callback(subsystemName));
  }

  static class Callback implements Consumer<SendableRegistry.CallbackData> {
    String subsystemName;

    Callback(String s) {
      this.subsystemName = s;
    }

    @Override
    public void accept(CallbackData t) {
      if (subsystemName == null || subsystemName.equals(t.subsystem)) {
        logger.info("- {} -> {}", t.subsystem, t.name);
      }
    }
  }

  public static class SlidingWindowStats {
    LinkedList<Double> values = new LinkedList<>();

    int maxSize;

    int size;

    double mean;
    double stdDev;

    int flyers;

    public SlidingWindowStats(int maxSize) {
      this.maxSize = maxSize;
      clear();
    }

    public void clear() {
      values.clear();
      size = 0;
      mean = 0.0;
      stdDev = 0.0;
      flyers = 0;
    }

    public void addValue(double v) {
      if (size == maxSize) {
        values.removeFirst();
        size--;
      }
      values.addLast(v);
      size++;

      updateStats();
    }

    void updateStats() {
      double sum = 0;
      for (var v : values) {
        sum += v;
      }
      mean = sum / values.size();

      sum = 0;
      for (var v : values) {
        sum += (v - mean) * (v - mean);
      }
      stdDev = sum / values.size();

      flyers = 0;
      for (var v : values) {
        if (Math.abs(v - mean) > (3 * stdDev)) {
          flyers++;
        }
      }
    }

    public int getSize() {
      return size;
    }

    public double getMean() {
      return mean;
    }

    public double getStdDev() {
      return stdDev;
    }

    public int getFlyers() {
      return flyers;
    }

    @Override
    public String toString() {
      return "SlidingWindowStats [size=" + size + ", mean=" + mean
          + ", stdDev=" + stdDev + ", flyers=" + flyers + "]";
    }

  }

}
