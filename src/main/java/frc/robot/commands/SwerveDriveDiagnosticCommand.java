// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.misc.Utilities.SlidingWindowStats;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.blinky.BlinkPattern;
import frc.robot.blinky.Pattern;
import frc.robot.blinky.SolidPattern;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

public class SwerveDriveDiagnosticCommand extends Command {
  LightSegment lightSegment;
  Logger logger = EventLogging.getLogger(getClass());
  List<CANSparkBase> azimuths = new ArrayList<>(4); // probably 4 motors
  Map<String, SlidingWindowStats> driveMotorCurrents = new HashMap<>();
  double power;

  Pattern patternOk = new SolidPattern().setColor(Color.kGreen);
  Pattern patternBad = new BlinkPattern().setColor(Color.kPink).setBlink(0.25);
  Pattern patternAziBad = new BlinkPattern().setColor(Color.kBlue).setBlink(.25);
  Pattern patternZeroMotors = new BlinkPattern().setColor(Color.kYellow).setBlink(.25);

  /** Creates a new SwerveDriveDiagnosticCommand. */
  public SwerveDriveDiagnosticCommand() {
    this(0.2);
  }

  public SwerveDriveDiagnosticCommand(double power) {
    this.power = power;
    this.lightSegment = RobotContainer.lightSegment;
    for (var nameAndMotor : RobotContainer.swerveDriveMotors.entrySet()) {
      if (nameAndMotor.getValue() instanceof CANSparkBase) {
        driveMotorCurrents.put(nameAndMotor.getKey(), new SlidingWindowStats(100));
      } else {
        logger.error("Swerve Drive Motor {} is not a CANSparkBase, it is a {}", nameAndMotor.getKey(),
            nameAndMotor.getValue().getClass());
      }
    }
    for (var nameAndMotor : RobotContainer.swerveAzimuthMotors.entrySet()) {
      if (nameAndMotor.getValue() instanceof CANSparkBase) {
        azimuths.add((CANSparkBase) nameAndMotor.getValue());
      } else {
        logger.error("Swerve Aizmuth Motor {} is not a CANSparkBase, it is a {}", nameAndMotor.getKey(),
            nameAndMotor.getValue().getClass());
      }
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.lightSegment, RobotContainer.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<Double> speeds = new ArrayList<>();
    List<Double> currents = new ArrayList<>();
    for (var nameAndMotor : RobotContainer.swerveDriveMotors.entrySet()) {
      if (nameAndMotor.getValue() instanceof CANSparkBase) {
        CANSparkBase motor = (CANSparkBase) nameAndMotor.getValue();
        motor.set(power);
        speeds.add(motor.getEncoder().getVelocity());
        SlidingWindowStats currentsStats = driveMotorCurrents.get(nameAndMotor.getKey());
        currentsStats.addValue(motor.getOutputCurrent());
        currents.add(currentsStats.getMean());
        SmartDashboard.putNumber("Diagnostics." + nameAndMotor.getKey() + ".drive.current", currentsStats.getMean());
      }
    }
    double minSpeed = Collections.min(speeds);
    double maxSpeed = Collections.max(speeds);
    double minCurrent = Collections.min(currents);
    double maxCurrent = Collections.max(currents);

    for (var azimuth : azimuths) {
      azimuth.set(power);
    }
    List<Double> aziSpeeds = new ArrayList<>(azimuths.size());
    List<Double> aziCurrents = new ArrayList<>(azimuths.size());
    for (var azimuth : azimuths) {
      aziSpeeds.add(azimuth.getEncoder().getVelocity());
      aziCurrents.add(azimuth.getOutputCurrent());
    }
    double minAziSpeed = Collections.min(aziSpeeds);
    double maxAziSpeed = Collections.max(aziSpeeds);
    double minAziCurrent = Collections.min(aziCurrents);
    double maxAziCurrent = Collections.max(aziCurrents);

    Pattern p = patternOk;
    // replace this with an expression to check the current
    if (minSpeed / maxSpeed < .9) {
      p = patternBad;
    }

    if (minAziSpeed / maxAziSpeed < .9) {
      p = patternAziBad;
    }

    // replace this with an expression to check the speeds
    if (minCurrent / maxCurrent < .9) {
      p = patternBad;
    }

    if (minAziCurrent / maxAziCurrent < .9) {
      p = patternAziBad;
    }

    if (maxAziSpeed == 0 || maxAziCurrent == 0 || maxCurrent == 0 || maxSpeed == 0) {
      p = patternZeroMotors;
    }

    // lightSegment.setPattern(p);
    SmartDashboard.putString("SwerveDriveDiagnosticCommand", p.toString());
    lightSegment.setPattern(patternAziBad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //for (var motor : motors) {
    //  motor.set(0.0);
    //}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
