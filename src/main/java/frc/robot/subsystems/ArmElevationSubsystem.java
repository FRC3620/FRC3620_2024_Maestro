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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElevationSubsystem extends SubsystemBase {
  final String name = "Arm Elevate";

  // PID parameters and encoder conversion factors
  final double kP = 0.4; //
  final double kI = 0;
  final double kD = 0;
  final double kFF = 0; // define FF
  final double outputLimit = 0.2; // the limit that the power cannot exceed

  final double positionConverionFactor = 1.0;
  final double velocityConverionFactor = 1.0;

  // Ingredients: Motor, Encoder, PID, and Timer
  CANSparkMaxSendable motor;
  RelativeEncoder motorEncoder;
  Timer calibrationTimer;

  SparkPIDController pid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;

  // saves the requested position
  double requestedPosition = 0;

  // to save a requested position if encoder is not calibrated
  Double requestedPositionWhileCalibrating = null;

  public ArmElevationSubsystem(CANSparkMaxSendable motor) { //The constructor
    this.motor = motor;
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
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);

    // only do something if we actually have a motor
    if (motor != null) { 
      SmartDashboard.putNumber(name + ".current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

      if (motorEncoder != null) { // and there is an encoder, display these
        double velocity = motorEncoder.getVelocity();
        double position = motorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".velocity", velocity);
        SmartDashboard.putNumber(name + ".position", position);

        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) { 
            // If the robot is running, and the encoder is "not calibrated," run motor very slowly towards the stop
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
                if (Math.abs(velocity) < 2) {
                  // the motor is not moving, stop the motor, set encoder position to 0, and set calibration to true
                  encoderCalibrated = true;
                  setPower(0.0);
                  motorEncoder.setPosition(0.0);

                  //If there was a requested position while we were calibrating, go there
                  if (requestedPositionWhileCalibrating != null) {
                    setPosition(requestedPositionWhileCalibrating);
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

  /**
   * Set the target position
   * @param position units are ???, referenced from position 0 == ?????
   */
  public void setPosition(double position) {
    position = MathUtil.clamp(position, -45, 200);
    SmartDashboard.putNumber(name + ".requestedPosition", position);
    requestedPosition = position;
    if (encoderCalibrated) {
      pid.setReference(position, ControlType.kPosition);
    } else {
      requestedPositionWhileCalibrating = position;
    }
  }

  /**
   * return the last requested position
   * @return the last requested position, units as in setPosition()
   */
  public double getRequestedPosition() {
    return requestedPosition;
  }

  /**
   * return the actual position
   * @return the current position
   */
  public double getActualPosition() {
    if (motorEncoder == null)
      return 0;
    double position = motorEncoder.getPosition();
    return position;
  }

  // Remember that power and position are different things. this should probably only
  // be used by the calibration routine in periodic()
  void setPower(double power) {
    motor.set(power);
  }

}
