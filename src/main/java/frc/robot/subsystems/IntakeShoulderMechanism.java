// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class IntakeShoulderMechanism implements HasTelemetry {
  final String name = "intake.shoulder";
  Logger logger = EventLogging.getLogger(getClass());

  // PID parameters and encoder conversion factors
  final double kP = 0.02; //
  final double kI = 0;
  final double kD = 0;
  final double kFF = 0.0; // define FF
  // final double outputLimit = 0.5; // the limit that the power cannot exceed

  final double positionConverionFactor = 1;
  final double velocityConverionFactor = positionConverionFactor;

  // Ingredients: Motor, Encoder, PID, and Timer
  CANSparkMax motor;
  RelativeEncoder motorEncoder;
  Timer calibrationTimer;

  //SparkPIDController pid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;

  // saves the requested location
  IntakeLocation requestedLocation = IntakeLocation.IntakeIn;

  // the requested pid setpoint
  double requestedPIDSetpoint;

  // in case we manually adjusting
  Double manualPower = null;

  boolean disabledForDebugging = false;

  public IntakeShoulderMechanism(CANSparkMax motor) { // The constructor
    this.motor = motor;
    if (motor != null) {
      /*
      pid = motor.getPIDController();
      pid.setP(kP); //
      pid.setI(kI); //
      pid.setD(kD); //
      pid.setFF(kFF); //
      pid.setOutputRange(-0.4, 0.4);
      */

      this.motorEncoder = motor.getEncoder();
      motorEncoder.setPositionConversionFactor(positionConverionFactor);
      motorEncoder.setVelocityConversionFactor(60);
    }
  }

  public void periodic() {
    // only do something if we actually have a motor
    if (motor != null) {
      if (motorEncoder != null) {
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) { 
            periodicWhileCalibrated();
          } else {
            periodicOnceCalibrated();
          }
        }
      }
    }
  }

  void periodicOnceCalibrated() {
    if (!disabledForDebugging) {
      Double requestedPower = null;
      if (manualPower != null) {
        requestedPower = manualPower;
      } else {
        double currentPosition = getActualPosition();
        if (requestedLocation == IntakeLocation.IntakeIn) {
          //Up
          if (currentPosition < 4) {
            //Close
            requestedPower = -0.02;
          } else {
            //Not Close
            requestedPower = -0.35;
          }
        } else {
          //Down
          if (currentPosition > 1.5) {
            //Close
            requestedPower = +0.01;
          } else {
            //Not Close
            requestedPower = +0.35;
          }
        }
      }

      if (requestedPower != null) {
        setPower(requestedPower);
        SmartDashboard.putNumber(name + ".requestedPower", requestedPower);
      } // else the PID is driving
    }
  }

  void periodicWhileCalibrated() {
    // If the robot is running, and the encoder is "not calibrated," run motor very
    // slowly towards the stop
    setPower(-0.1);
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
  }

  public void markCalibrated() {
    // the motor is not moving, stop the motor, set encoder position to 0, and set
    // calibration to true
    encoderCalibrated = true;
    setPower(0.0);
    motorEncoder.setPosition(0);
    setLocation(IntakeLocation.IntakeIn);
  }

  public void setManualPower(Double power) {
    this.manualPower = power;
  }

  public boolean isCalibrated() {
    return encoderCalibrated;
  }

  public void setLocation(IntakeLocation location) {
    this.requestedLocation = location;
    setPositionSetpoint(location.getIntakePositionSetpoint());
  }

  public IntakeLocation getRequestedLocation() {
    return requestedLocation;
  }

  /**
   * Set the target position
   * 
   * @param position units are ???, referenced from position 0 == in
   */
  void setPositionSetpoint(double position) {
    SmartDashboard.putNumber(name + ".requestedPosition", position);
    requestedPIDSetpoint = position;
    //pid.setReference(position, ControlType.kPosition);
  }

  /**
   * return the last requested position
   * 
   * @return the last requested position, units as in setPosition()
   */
  public double getPositionSetpoint() {
    return requestedPIDSetpoint;
  }

  /**
   * return the actual position
   * 
   * @return the current position
   */
  public double getActualPosition() {
    if (motorEncoder != null) {
    return motorEncoder.getPosition();
    } else {
      return 0.0;
    }
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

  @Override
  public void updateTelemetry() {
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);

    SmartDashboard.putString(name + ".requestedLocation", requestedLocation.toString());

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
  }

}