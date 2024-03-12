// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
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
  List<CANSparkBase> motors = new ArrayList<>(4); // probably 4 motors
  double power;

  Pattern patternOk = new SolidPattern().setColor(Color.kGreen);
  Pattern patternBad = new BlinkPattern().setColor(Color.kRed).setBlink(0.25);

  /** Creates a new SwerveDriveDiagnosticCommand. */
  public SwerveDriveDiagnosticCommand() {
    this (0.2);
  }

  public SwerveDriveDiagnosticCommand(double power) {
    this.power = power;
    this.lightSegment = RobotContainer.lightSegment;
    for (var nameAndMotor : RobotContainer.swerveDriveMotors.entrySet()) {
      if (nameAndMotor.getValue() instanceof CANSparkBase) {
        motors.add((CANSparkBase) nameAndMotor.getValue());
      } else {
        logger.error ("Swerve Drive Motor {} is not a CANSparkBase, it is a {}", nameAndMotor.getKey(), nameAndMotor.getValue().getClass());
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
    for (var motor : motors) {
      motor.set(power);
    }
    List<Double> speeds = new ArrayList<>(motors.size());
    List<Double> currents = new ArrayList<>(motors.size());
    for (var motor : motors) {
      speeds.add(motor.getEncoder().getVelocity());
      currents.add(motor.getOutputCurrent());
    }
    double minSpeed = Collections.min(speeds);
    double maxSpeed = Collections.max(speeds);
    double minCurrent = Collections.min(currents);
    double maxCurrent = Collections.max(currents);

    Pattern p = patternOk;
    // replace this with an expression to check the current
    if (false) {
      p = patternBad;
    }
    // replace this with an expression to check the speeds
    if (false) {
      p = patternBad;
    }
    lightSegment.setPattern(p);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (var motor : motors) {
      motor.set(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
