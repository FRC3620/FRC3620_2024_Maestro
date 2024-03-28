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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.blinky.BlinkPattern;
import frc.robot.subsystems.BlinkySubsystem.LightSegment;

public class SwerveDriveDiagnosticCommand extends Command {
  final int slidingWindowSize = 100; // 50 samples a second, so 2 seconds
  LightSegment lightSegment;
  Logger logger = EventLogging.getLogger(getClass());

  Map<String, SlidingWindowStats> driveMotorCurrents = new HashMap<>();
  Map<String, SlidingWindowStats> azimuthMotorCurrents = new HashMap<>();

  double power;

  Color colorOk = Color.kGreen;
  Color colorDriveBad = Color.kPink;
  Color colorAziBad = Color.kBlue;
  Color colorZeroMotors = Color.kYellow;

  BlinkPattern myPattern = new BlinkPattern().setColor1(Color.kBlack).setColor2(Color.kBlack).setBlinkTime(0.5);

  /** Creates a new SwerveDriveDiagnosticCommand. */
  public SwerveDriveDiagnosticCommand() {
    this(0.2);
  }

  public SwerveDriveDiagnosticCommand(double power) {
    this.power = power;
    this.lightSegment = RobotContainer.lightSegment;

    for (var nameAndMotor : RobotContainer.swerveDriveMotors.entrySet()) {
      if (nameAndMotor.getValue().getMotor() instanceof CANSparkBase) {
        driveMotorCurrents.put(nameAndMotor.getKey(), new SlidingWindowStats(slidingWindowSize));
      } else {
        logger.error("Swerve Drive Motor {} is not a CANSparkBase, it is a {}", nameAndMotor.getKey(),
            nameAndMotor.getValue().getClass());
      }
    }

    for (var nameAndMotor : RobotContainer.swerveAzimuthMotors.entrySet()) {
      if (nameAndMotor.getValue().getMotor() instanceof CANSparkBase) {
        azimuthMotorCurrents.put(nameAndMotor.getKey(), new SlidingWindowStats(slidingWindowSize));
      } else {
        logger.error("Swerve Azimuth Motor {} is not a CANSparkBase, it is a {}", nameAndMotor.getKey(),
            nameAndMotor.getValue().getClass());
      }
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.lightSegment, RobotContainer.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightSegment.setPattern(myPattern);

    for (var stat : driveMotorCurrents.values()) {
      stat.clear();
    }
    for (var stat : azimuthMotorCurrents.values()) {
      stat.clear();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Color driveColor = colorOk;
    if (driveMotorCurrents.size() > 0) {
      List<Double> driveSpeeds = new ArrayList<>(driveMotorCurrents.size());
      List<Double> driveCurrents = new ArrayList<>(driveMotorCurrents.size());
      for (var nameAndCurrentStat : driveMotorCurrents.entrySet()) {
        String name = nameAndCurrentStat.getKey();
        SlidingWindowStats currentStat = nameAndCurrentStat.getValue();
        CANSparkBase motor = (CANSparkBase) RobotContainer.swerveDriveMotors.get(name).getMotor();
        motor.set(power);

        double velocity = motor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Diagnostics." + name + ".drive.speed", velocity);
        driveSpeeds.add(velocity);

        currentStat.addValue(motor.getOutputCurrent());
        double mean = currentStat.getMean();
        SmartDashboard.putNumber("Diagnostics." + name + ".drive.current", mean);
        driveCurrents.add(mean);
      }

      double minDriveSpeed = Collections.min(driveSpeeds);
      double maxDriveSpeed = Collections.max(driveSpeeds);
      double minDriveCurrent = Collections.min(driveCurrents);
      double maxDriveCurrent = Collections.max(driveCurrents);

      if (minDriveSpeed / maxDriveSpeed < .9) {
        driveColor = colorDriveBad;
      }

      if (minDriveCurrent / maxDriveCurrent < .9) {
        driveColor = colorDriveBad;
      }

      if (maxDriveCurrent == 0 || maxDriveSpeed == 0) {
        driveColor = colorZeroMotors;
      }
    } else {
      driveColor = colorZeroMotors;

    }
    myPattern.setColor1(driveColor);

    Color aziColor = colorOk;
    if (azimuthMotorCurrents.size() > 0) {
      List<Double> aziSpeeds = new ArrayList<>(azimuthMotorCurrents.size());
      List<Double> aziCurrents = new ArrayList<>(azimuthMotorCurrents.size());
      for (var nameAndCurrentStat : azimuthMotorCurrents.entrySet()) {
        String name = nameAndCurrentStat.getKey();
        SlidingWindowStats currentStat = nameAndCurrentStat.getValue();
        CANSparkBase motor = (CANSparkBase) RobotContainer.swerveAzimuthMotors.get(name).getMotor();
        motor.set(power);

        double velocity = motor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Diagnostics." + name + ".azimuth.speed", velocity);
        aziSpeeds.add(velocity);

        currentStat.addValue(motor.getOutputCurrent());
        double mean = currentStat.getMean();
        SmartDashboard.putNumber("Diagnostics." + name + ".azimuth.current", mean);
        aziCurrents.add(mean);
      }

      double minAziSpeed = Collections.min(aziSpeeds);
      double maxAziSpeed = Collections.max(aziSpeeds);
      double minAziCurrent = Collections.min(aziCurrents);
      double maxAziCurrent = Collections.max(aziCurrents);

      if (minAziSpeed / maxAziSpeed < .9) {
        aziColor = colorAziBad;
      }

      if (minAziCurrent / maxAziCurrent < .9) {
        aziColor = colorAziBad;
      }

      if (maxAziSpeed == 0 || maxAziCurrent == 0) {
        aziColor = colorZeroMotors;
      }
    } else {
      aziColor = colorZeroMotors;
    }
    myPattern.setColor2(aziColor);

    SmartDashboard.putString("Diagnostics.lightPattern", myPattern.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
