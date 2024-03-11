// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.blinky;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

public class DefaultBlinkyCommand extends Command {
  RollersSubsystem rollersSubsystem = RobotContainer.rollersSubsystem;
  VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

  final Pattern blinkyGreen = new BlinkPattern().setColor(Color.kGreen);
  final Pattern solidGreen = new SolidPattern().setColor(Color.kGreen);
  final Pattern solidRed = new SolidPattern().setColor(Color.kRed);
  final Pattern blinkyRed = new BlinkPattern().setColor(Color.kRed).setOnSeconds(0.1).setOffSeconds(1.0);

  LightSegment lightSegment;

  public DefaultBlinkyCommand(LightSegment lightSegment) {
    this.lightSegment = lightSegment;
    addRequirements(lightSegment);
  }

  @Override
  public void execute() {
    if (Robot.getCurrentRobotMode() == RobotMode.DISABLED) {
      lightSegment.setPattern(blinkyRed);
    } else {
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
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
