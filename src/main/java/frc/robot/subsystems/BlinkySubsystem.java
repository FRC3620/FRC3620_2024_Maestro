// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.blinky.BlinkPattern;
import frc.robot.blinky.LightSegment;
import frc.robot.blinky.Pattern;
import frc.robot.blinky.SolidPattern;

/**/
public class BlinkySubsystem extends SubsystemBase {

  AddressableLED leds;
  AddressableLEDBuffer lBuffer;
  Timer timer;
  public LightSegment lightSegment = new LightSegment(0, 19);
  // final int numberOfLeds= 20;

  RollersSubsystem rollersSubsystem;
  VisionSubsystem visionSubsystem;

  /** Creates a new BlinkySubsystem. */
  public BlinkySubsystem(RollersSubsystem rollersSubsystem, VisionSubsystem visionSubsystem) {
    this.rollersSubsystem = rollersSubsystem;
    this.visionSubsystem = visionSubsystem;

    leds = new AddressableLED(0);
    lBuffer = new AddressableLEDBuffer(19);
    timer = new Timer();
    leds.setLength(lBuffer.getLength());
    leds.start();
  }

  public void updateLEDS(int first, int last, Color color) {
    for (int i = first; i < last; i++) {
      lBuffer.setLED(i, color);
    }
  }

  /*
   * public void setChase(int first,int last,Color color,int speed){
   * timer.reset();
   * timer.start();
   * for(int i= first;i<last;i++){
   * lBuffer.setLED(i, color);
   * lBuffer.setLED(i+1, color);
   * lBuffer.setLED(i+2, color);
   * // lBuffer.setLED(i+3, color);
   * }
   * }
   */

  final Pattern blinkyGreen = new BlinkPattern().setColor(Color.kGreen);
  final Pattern solidGreen = new SolidPattern().setColor(Color.kGreen);
  final Pattern solidRed = new SolidPattern().setColor(Color.kRed);

  @Override
  public void periodic() {
    // If game piece detected and we have a shooting solution, blink green
    if (rollersSubsystem.gamePieceDetected()) {
      if (visionSubsystem.doIHaveShootingSolution()) {
        lightSegment.setPattern(blinkyGreen);
      } else {
        // If we have gamepiece but no shooting solution, solid green
        lightSegment.setPattern(solidGreen);
      }
    } else { // No game piece, solid red
      lightSegment.setPattern(solidRed);
    }
    lightSegment.periodic();
    leds.setData(lBuffer);
  }
}
