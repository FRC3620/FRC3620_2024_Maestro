// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShooterElevationSubsystem extends SubsystemBase implements HasTelemetry {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  CANDeviceFinder deviceFinder = RobotContainer.canDeviceFinder;

  public CANSparkMaxSendable elevationMotor;
  RelativeEncoder elevationMotorEncoder;

  public final static int MOTORID_SHOOTER_ELEVATION = 16;

  SparkPIDController elevationPid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;
  Timer calibrationTimer;

  // saves the requested position
  double requestedPosition = 0;

  // to save a requested position if encoder is not calibrated
  Double requestedPositionWhileCalibrating = null;

  boolean disabledForDebugging = false;

  final String name = "shooter";

  double elevationAdjustment = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterElevationSubsystem() {
    // shooter angle PID parameters and encoder conversion factors
    final double kP = 0.0175; //0.02
    final double kI = 0.0; //0.00007
    final double kD = 0;
    final double kFF = 0;
    final double kIMaxAccum = 0.04;
    final double negOutputLimit = -0.2;
    final double posOutputLimit = 0.2;

    // was 9.44 before we swapped out 3:1/5:1 for a 5:1/5:1
    // final double positionConverionFactor = 1.0 * 9.44 * (15.0 / 25.0);
    final double positionConverionFactor = (64 - 18.1) / (64 - 54.93); // difference in angle divided by difference in
                                                                       // motor rotation
    final double velocityConverionFactor = 1.0;

    if (deviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_SHOOTER_ELEVATION, "Shooter Elaevation")
        || RobotContainer.shouldMakeAllCANDevices()) {
      elevationMotor = new CANSparkMaxSendable(MOTORID_SHOOTER_ELEVATION, MotorType.kBrushless);
      MotorSetup setup = new MotorSetup().setCurrentLimit(10).setCoast(false);
      setup.apply(elevationMotor);
      addChild("elevationMotor", elevationMotor);

      MotorSetup motorSetup = new MotorSetup().setCoast(false).setCurrentLimit(80);
      motorSetup.apply(elevationMotor);
      elevationMotorEncoder = elevationMotor.getEncoder();
      elevationMotorEncoder.setPositionConversionFactor(positionConverionFactor);
      elevationMotorEncoder.setVelocityConversionFactor(velocityConverionFactor);

      elevationPid = elevationMotor.getPIDController();
      elevationPid.setP(kP);
      elevationPid.setI(kI);
      elevationPid.setD(kD);
      elevationPid.setFF(kFF);
      var err = elevationPid.setIMaxAccum(kIMaxAccum, 0);
      if (err != REVLibError.kOk) {
        logger.error("Could not set elevation kImaxAccum: {}", err);
      }
      elevationPid.setOutputRange(negOutputLimit, posOutputLimit);
    }
    
    updateDashboardElevationAdjustment();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("shooterSubsystemCommand", "" + getCurrentCommand());
    double newElevationAdjustment = readDashboardElevationAdjustment();
    if (newElevationAdjustment != elevationAdjustment) {
      elevationAdjustment = newElevationAdjustment;
      if (encoderCalibrated && ! disabledForDebugging) {
        elevationPid.setReference(requestedPosition + elevationAdjustment, ControlType.kPosition);
      }
    }

    // only do something if we actually have a motor
    if (elevationMotor != null) {
      if (elevationMotorEncoder != null) { // if there is an encoder, display these
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) {
            // If the robot is running, and the encoder is "not calibrated," run motor very
            // slowly towards the switch
            setElevationPower(0.08); 
            if (calibrationTimer == null) {
              // we need to calibrate and we have no timer. make one and start it
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              // we have a timer, has the motor had power long enough to spin up
              if (calibrationTimer.get() > 0.5) {
                if (Math.abs(elevationMotorEncoder.getVelocity()) < 1) {
                  // motor is not moving, hopefully it's against the stop
                  encoderCalibrated = true;
                  setElevationPower(0.0);
                  elevationMotorEncoder.setPosition(68.5);
                  setElevationPosition(68.5);

                  // If there was a requested position while we were calibrating, go there
                  if (requestedPositionWhileCalibrating != null) {
                    setElevationPosition(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }

                }
              }
            }
          }
        }
      }
    }
  }

  public void setElevationPosition(double position) {
    position = MathUtil.clamp(position, 20, 68.5); // high end is a little short of the stop
    SmartDashboard.putNumber(name + ".elevation.requestedPosition", position);
    requestedPosition = position;
    if (encoderCalibrated) {
      if (!disabledForDebugging) {
        elevationPid.setReference(position + elevationAdjustment, ControlType.kPosition);
      }
    } else {
      requestedPositionWhileCalibrating = position;
    }
  }

  public double getRequestedShooterElevation() {
    return requestedPosition;
  }

  public double getActualElevationPosition() {
    if (elevationMotorEncoder != null) return elevationMotorEncoder.getPosition();
    return 0;
  }

  void setElevationPower(double power) {
    if (!disabledForDebugging) {
      elevationMotor.set(power);
    }
  }

  public void setElevationPosition(ShooterSpeedAndAngle speedAndAngle) {
    setElevationPosition(speedAndAngle.position);
  }

  @Override
  public void updateTelemetry() {
    SmartDashboard.putBoolean(name + ".elevation.calibrated", encoderCalibrated);
    if (elevationMotor != null) {
      SmartDashboard.putNumber(name + ".elevation.current", elevationMotor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".elevation.power", elevationMotor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".elevation.temperature", elevationMotor.getMotorTemperature());
      SmartDashboard.putNumber(name + ".elevation.IAccum", elevationPid.getIAccum());

      if (elevationMotorEncoder != null) { // if there is an encoder, display these
        double velocity = elevationMotorEncoder.getVelocity();
        double position = elevationMotorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".elevation.velocity", velocity);
        SmartDashboard.putNumber(name + ".elevation.position", position);
      }
    }
  }

  void updateDashboardElevationAdjustment() {
    SmartDashboard.putNumber(name + ".elevation.adjustment", elevationAdjustment);
  }

  double readDashboardElevationAdjustment() {
    return SmartDashboard.getNumber(name + ".elevation.adjustment", elevationAdjustment);
  }

  public double getElevationAdjustment() {
    return elevationAdjustment;
  }

}
