// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.Utilities;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElevationSubsystem extends SubsystemBase {
  /** Creates a new ArmElevationSubsystem. */

  // Ingredients: Motor, Encoder, PID, and Timer
  CANSparkMaxSendable motor;
  RelativeEncoder motorEncoder;
  Timer calibrationTimer;

  SparkPIDController pid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;
  int elevateEncoderValueAt90Degrees;

  Double requestedElevationWhileCalibrating = null; // to save a requested position if robot is not calibrated

  final double kP = 0.4; // change PID later
  final double kI = 0;
  final double kD = 0;
  final double kFF = 0;

  final double outputLimit = 0.2;

  double elevationOffset;

  double requestedElevation = 0; // degrees change later

  final String name = "Arm Elevate";

  public ArmElevationSubsystem(CANSparkMaxSendable motor) {

    this.motor = motor;
    this.motorEncoder = motor.getEncoder();

    pid = motor.getPIDController();

    pid.setP(kP); // 0.1
    pid.setI(kI); // 0.0
    pid.setD(kD); // 10
    pid.setFF(kFF); // 0.0

    pid.setOutputRange(-outputLimit, outputLimit);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);

    if (motor != null) { //If there is a motor, display these
      SmartDashboard.putNumber(name + ".current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

      if (motorEncoder != null) { // and there is an encoder, display these
        double velocity = motorEncoder.getVelocity();
        double elevation = motorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".speed", velocity);
        SmartDashboard.putNumber(name + ".elevation", elevation);

        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) { 
            setPower(0.03); //If the robot is running, and the encoder is "not calibrated," run motor very slowly

            if (calibrationTimer == null) { //If there is no timer, make one
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {//When there is a timer, wait for 0.5 seconds
              if (calibrationTimer.get() > 0.5) {
                if (Math.abs(velocity) < 2) {
                  //Then, if the motor is not moving, stop the motor, set encoder position to 0, and set calibration to true
                  encoderCalibrated = true;
                  setPower(0.0);
                  motorEncoder.setPosition(0.0);
                  if (requestedElevationWhileCalibrating != null) { //If there was a requested position, go there
                    setElevation(requestedElevationWhileCalibrating);
                    requestedElevationWhileCalibrating = null;
                  } else {
                    // this might try to extend arm while vertical, a big no-no!
                    // setLength(encoder.getPosition());
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  public void setElevation(double elevation) {
    elevation = MathUtil.clamp(elevation, -45, 200);
    SmartDashboard.putNumber(name + ".requestedElevation", elevation);
    requestedElevation = elevation;
    if (encoderCalibrated) {
      pid.setReference(elevation, ControlType.kPosition);
    } else {
      requestedElevationWhileCalibrating = elevation;
    }
  }

  public double getRequestedElevation() {
    return requestedElevation;
  }

  public void setPower(double power) {
    motor.set(power);
  }

  public double getCurrentElevation() {
    if (motorEncoder == null)
      return 0;
    double motorEncoderValue = motorEncoder.getPosition();
    // converting heading from tics (ranging from 0 to 4095) to degrees
    double elevation = (motorEncoderValue - elevateEncoderValueAt90Degrees) * (360.0 / 4096.0) + 90;
    // get it into the -180..180 range
    elevation = Utilities.normalizeAngle(elevation);
    // get it into the -90..270 range
    if (elevation < -90) {
      elevation = elevation + 360;
    }
    return elevation;
  }
} // Remember that power and elevation are different things
