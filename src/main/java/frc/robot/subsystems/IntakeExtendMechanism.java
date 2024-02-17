// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class IntakeExtendMechanism {
  final String name = "intake.extend";

  // PID parameters and encoder conversion factors
  final double kP = 0.2; //
  final double kI = 0;
  final double kD = 0;
  final double kFF = 0; // define FF
  final double outputLimit = 0.1; // the limit that the power cannot exceed

  final double positionConverionFactor = 1.0;
  final double velocityConverionFactor = 1.0;

  // Ingredients: Motor, Encoder, PID, and Timer
  CANSparkMaxSendable motor;
  CANSparkMaxSendable motor2;
  RelativeEncoder motorEncoder;
  RelativeEncoder motor2Encoder;
  Timer calibrationTimer;

  SparkPIDController pid = null;
  SparkPIDController pid2 = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;

  // saves the requested position
  double requestedPosition = 0;
  double temporaryPosition = 0;

  // to save a requested position if encoder is not calibrated
  Double requestedPositionWhileCalibrating = null;

  boolean disabledForDebugging = true;

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
      motor2Encoder = motor2.getEncoder();
      motor2Encoder.setPositionConversionFactor(positionConverionFactor);
      motor2Encoder.setVelocityConversionFactor(velocityConverionFactor);

      pid2 = motor2.getPIDController();
      pid2.setP(kP); //
      pid2.setI(kI); //
      pid2.setD(kD); //
      pid2.setFF(kFF); //
      pid2.setOutputRange(-outputLimit, outputLimit);
    }
  }

  public void periodic() {
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);

    // only do something if we actually have a motor
    if (motor != null && motor2 != null) {
      SmartDashboard.putNumber(name + ".current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());
      SmartDashboard.putNumber(name + "2.current", motor2.getOutputCurrent());
      SmartDashboard.putNumber(name + "2.power", motor2.getAppliedOutput());
      SmartDashboard.putNumber(name + "2.temperature", motor2.getMotorTemperature());
      

      SmartDashboard.putBoolean("outOfWhack", outOfWhack());

      if (motorEncoder != null) { // and there is an encoder, display these
        double velocity = motorEncoder.getVelocity();
        double position = motorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".velocity", velocity);
        SmartDashboard.putNumber(name + ".position", position);

        if (motor2Encoder != null) { // and there is an encoder, display these
          double velocity2 = motor2Encoder.getVelocity();
          double position2 = motor2Encoder.getPosition();
          SmartDashboard.putNumber(name + "2.velocity", velocity2);
          SmartDashboard.putNumber(name + "2.position", position2);
        }
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) {
            // If the robot is running, and the encoder is "not calibrated," run motor very
            // slowly towards the stop
            setPower(0.03);
            setPower2(.03);

            if (calibrationTimer == null) {
              // we need to calibrate and we have no timer. make one and start it
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              // we have a timer, has the motor had power long enough to spin up
              if (calibrationTimer.get() > 0.5) {
                // motor should be moving if not against the stop
                if (Math.abs(velocity) < 2) {
                  // the motor is not moving, stop the motor, set encoder position to 0, and set
                  // calibration to true
                  encoderCalibrated = true;
                  setPower(0.0);
                  setPower2(0.0);
                  motorEncoder.setPosition(0.0);
                  motor2Encoder.setPosition(0.0);

                  // If there was a requested position while we were calibrating, go there
                  if (requestedPositionWhileCalibrating != null) {
                    setPosition(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }
                }
              }
            }
          } else {

            // we are calibrated. Do magic here

            // tell motor2 to go to where motor1 currently is
            if (pid2 != null) {
              setPIDReference2(motorEncoder.getPosition());
            }
            // not
            if (outOfWhack()) {
              // figure out where motor1 is
              temporaryPosition = motorEncoder.getPosition();
              if (pid != null) {
                // tell motor1 to stay where it is right now
                setPIDReference(temporaryPosition);
              }
            } else {
              // we are in whack
              if (pid != null) {
                // tell motor1 to go to the position requested
                 setPIDReference(requestedPosition);
              }
            }

          }
        }
      }
    }
  }

  // this will return "out of wack" if motor 2 gets too behind motor 1
  boolean outOfWhack() {
    double position2;
    double position1;
    position1 = motorEncoder.getPosition();
    position2 = motor2Encoder.getPosition();
    SmartDashboard.putNumber("extend position difference", Math.abs(position2 - position1));
    if (Math.abs(position2 - position1) > 1) {
      // change 1 to actual restraint
      return true;
    } else {
      return false;
    }
  }

  /**
   * Set the target position
   * 
   * @param position units are ???, referenced from position 0 == ?????
   */
  public void setPosition(double position) {
    position = MathUtil.clamp(position, -45, 200);
    SmartDashboard.putNumber(name + ".requestedPosition", position);
    requestedPosition = position;
    if (encoderCalibrated) {
      // pid.setReference(teposition, ControlType.kPosition);
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

  void setPower2(double power) {
    if (motor2 != null) {
      if (!disabledForDebugging) {
        motor2.set(power);
      }
    }
  }

  void setPIDReference(double position) {
    if (!disabledForDebugging) {
      pid.setReference(position, ControlType.kPosition);
    }
    SmartDashboard.putNumber("position motor1", position);
  }

  void setPIDReference2(double position) {
    if (!disabledForDebugging) {
      pid2.setReference(position, ControlType.kPosition);
    }
    SmartDashboard.putNumber("position motor2", position);
  }
}