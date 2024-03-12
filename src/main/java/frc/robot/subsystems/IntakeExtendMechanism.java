// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class IntakeExtendMechanism implements HasTelemetry {
  final String name = "intake.extend";

  // PID parameters and encoder conversion factors
  final double kP = 0.1; //
  final double kI = 0;
  final double kD = 0;
  final double kFF = 0; // define FF
  final double outputLimit = 1; // the limit that the power cannot exceed

  final double positionConverionFactor = 3.14159 * 1.625 / 9; // 1 5/8" diameter, moves one circumference for every
                                                              // motor revolution, 1:9
  final double velocityConverionFactor = 1.0;

  // Ingredients: Motor, Encoder, PID, and Timer
  CANSparkMaxSendable motor;
  CANSparkMaxSendable motor2;
  RelativeEncoder motorEncoder;
  RelativeEncoder motor2Encoder;
  Timer calibrationTimer;

  SparkPIDController pid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;

  // saves the requested position
  Double requestedPosition = null;
  double temporaryPosition = 0;

  // to save a requested position if encoder is not calibrated
  Double requestedPositionWhileCalibrating = null;

  boolean disabledForDebugging = false;

  public boolean oldWay = false;

  public IntakeExtendMechanism(CANSparkMaxSendable motor, CANSparkMaxSendable motor2) { // The constructor
    this.motor = motor;
    this.motor2 = motor2;
    if (motor != null) {
      motorEncoder = motor.getEncoder();
      motorEncoder.setPositionConversionFactor(positionConverionFactor);
      motorEncoder.setVelocityConversionFactor(velocityConverionFactor);

      pid = motor.getPIDController();
      pid.setP(kP); //
      pid.setI(kI); //
      pid.setD(kD); //
      pid.setFF(kFF); //
      pid.setOutputRange(-outputLimit, outputLimit);

    }

    if (motor2 != null) {
      if (motor != null) {
        motor2.follow(motor, true); // TODO check sign!
      }
      motor2Encoder = motor2.getEncoder();
      motor2Encoder.setPositionConversionFactor(positionConverionFactor);
      motor2Encoder.setVelocityConversionFactor(velocityConverionFactor);
    }
  }

  public void periodic() {
    // only do something if we actually have a motor
    if (motor != null && motor2 != null) {
      if (motorEncoder != null) {
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) {
            // If the robot is enabled, and we are "not calibrated," run motors very slowly
            // towards the stop
            setPower(0.03);

            if (calibrationTimer == null) {
              // we need to calibrate and we have no timer. make one and start it
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              // we have a timer, has the motor had power long enough to spin up
              if (calibrationTimer.get() > 0.5) {
                // motor should be moving if not against the stop
                double velocity = motorEncoder.getVelocity();
                if (Math.abs(velocity) < 2) {
                  markCalibrated();
                }
              }
            }
          } else {
            // we are calibrated. Do magic here
            if (requestedPosition != null) {
              setPIDReference(requestedPosition);
            } else {
              motor.stopMotor();
            }
          }
        }
      }
    }
  }

  public void markCalibrated() {
    // stop the motor, set encoder position to 0, and set calibration to true
    encoderCalibrated = true;
    setPower(0.0);
    motorEncoder.setPosition(0.0);
    motor2Encoder.setPosition(0.0);

    // If there was a requested position while we were calibrating, go there
    if (requestedPositionWhileCalibrating != null) {
      setPosition(requestedPositionWhileCalibrating);
      requestedPositionWhileCalibrating = null;
    }
  }

  public boolean isCalibrated() {
    return encoderCalibrated;
  }

  // this will return "out of wack" if motor 2 gets too behind motor 1
  boolean outOfWhack() {
    double position2;
    double position1;
    if (motorEncoder != null && motor2Encoder != null) {
      position1 = motorEncoder.getPosition();
      position2 = motor2Encoder.getPosition();
      SmartDashboard.putNumber(name + ".whackiness", Math.abs(position2 - position1));
      if (Math.abs(position2 - position1) > 1) {
        // change 1 to actual restraint
        return true;
      } else {
        return false;
      }
    }else{
      return false;
    }
  }

  /**
   * Set the target position
   * 
   * @param position units are ???, referenced from position 0 == ?????
   */
  public void setPosition(Double position) {
    SmartDashboard.putNumber(name + ".requestedPosition", position != null ? position : 3620);
    if (position != null) {
      position = MathUtil.clamp(position, 0, 19);
    }
    requestedPosition = position;
    if (encoderCalibrated) {
      // setPIDReference(position);
    } else {
      requestedPositionWhileCalibrating = position;
    }
  }

  /**
   * return the last requested position
   * 
   * @return the last requested position, units as in setPosition()
   */
  public double getRequestedPosition() {
    return requestedPosition;
  }

  /**
   * return the actual position
   * 
   * @return the current position
   */
  public double getActualPosition() {
    if (motorEncoder == null)
      return 0;
    double position = motorEncoder.getPosition();
    return position;
  }

  // Remember that power and position are different things. this should probably
  // only
  // be used by the calibration routine in periodic()
  void setPower(double power) {
    if (motor != null) {
      if (!disabledForDebugging) {
        motor.set(power);
      }
    }
  }

  void setPIDReference(double position) {
    if (!disabledForDebugging) {
      pid.setReference(position, ControlType.kPosition);
    }
    SmartDashboard.putNumber(name + ".pidSetPoint", position);
  }

  @Override
  public void updateTelemetry() {
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);
    if (motor != null) {
      SmartDashboard.putNumber(name + ".current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());
      if (motorEncoder != null) { // and there is an encoder, display these
        double velocity = motorEncoder.getVelocity();
        double position = motorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".velocity", velocity);
        SmartDashboard.putNumber(name + ".position", position);
      }
    }
    if (motor2 != null) {
      SmartDashboard.putNumber(name + "2.current", motor2.getOutputCurrent());
      SmartDashboard.putNumber(name + "2.power", motor2.getAppliedOutput());
      SmartDashboard.putNumber(name + "2.temperature", motor2.getMotorTemperature());
      if (motor2Encoder != null) { // and there is an encoder, display these
        double velocity2 = motor2Encoder.getVelocity();
        double position2 = motor2Encoder.getPosition();
        SmartDashboard.putNumber(name + "2.velocity", velocity2);
        SmartDashboard.putNumber(name + "2.position", position2);
      }
    }
    SmartDashboard.putBoolean(name + ".inWhack", !outOfWhack());
  }
}