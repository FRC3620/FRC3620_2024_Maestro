// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.blinky;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

public class DefaultBlinkyCommand extends Command {
  IndexerSubsystem indexerSubsystem = RobotContainer.indexerSubsystem;
  VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

  final Pattern patternReadyToShoot = new BlinkPattern().setColor(Color.kGreen);
  final Pattern patternGotPiece = new SolidPattern().setColor(Color.kGreen);
  final Pattern patternNoPiece = new SolidPattern().setColor(Color.kGray);
  final Pattern patternIdle = new BlinkPattern().setColor(Color.kGreen).setOnSeconds(0.1).setOffSeconds(1.0);
  // final Pattern patternSick = new
  // ChasePattern().setColor1(Color.kRed).setColor2(Color.kBlue).setBlinkTime(0.25);
  final Pattern patternSick = new BlinkPattern().setColor1(Color.kRed).setColor2(Color.kBlue).setBlinkTime(0.50);
  final Pattern patternReallySick = new BlinkPattern().setColor1(Color.kRed).setColor2(Color.kBlue).setBlinkTime(0.10);

  LightSegment lightSegment;

  public DefaultBlinkyCommand(LightSegment lightSegment) {
    this.lightSegment = lightSegment;
    addRequirements(lightSegment);
  }

  @Override
  public void execute() {
    Pattern p;
    if (Robot.getCurrentRobotMode() == RobotMode.DISABLED) {
      switch (RobotContainer.healthMonitorSubsystem.getHealth()) {
        case SICK:
          p = patternSick;
          break;

        case REALLY_SICK:
          p = patternReallySick;
          break;

        default:
          p = patternIdle;
          break;
      }
    } else {
      if (indexerSubsystem.gamePieceDetected()) {
        if (visionSubsystem.doIHaveShootingSolution()) {
          p = patternReadyToShoot;
        } else {
          // If we have gamepiece but no shooting solution, solid green
          p = patternGotPiece;
        }
      } else { // No game piece, solid gray
        p = patternNoPiece;
      }

      if (!DriverStation.isFMSAttached()) {
        switch (RobotContainer.healthMonitorSubsystem.getHealth()) {
          case SICK:
            p = patternSick;
            break;

          case REALLY_SICK:
            p = patternReallySick;
            break;

          default:
            break;
        }
      }

    }
    lightSegment.setPattern(p);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
