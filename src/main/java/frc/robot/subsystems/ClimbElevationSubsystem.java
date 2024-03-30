// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbElevationSubsystem extends SubsystemBase implements HasTelemetry {
  final String name = "climber";

  // PID parameters and encoder conversion factors
  final double kP = 0.4; //
  final double kI = 0;
  final double kD = 0;
  final double kFF = 0; // define FF
  final double negOutputLimit = -0.75; // the limit that the power cannot exceed
  final double posOutputLimit = 0.85; // the limit that the power cannot exceed

  final double positionConverionFactor = 0.11962 / 3; // was 0.11962, we just threw another 3:1 in
  final double velocityConverionFactor = 60.0;

  //public static ClimbElevationSubsystem climbPosition = setPosition();

  // Ingredients: Motor, Encoder, PID, and Timer
  public CANSparkMaxSendable motor;
  RelativeEncoder motorEncoder;
  DigitalInput limitSwitch;
  Timer calibrationTimer;

  SparkPIDController pid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;

  // saves the requested position
  double requestedPosition = 0;

  // saves the last requested power, if any
  Double requestedPower = null;

  // to save a requested position if encoder is not calibrated
  Double requestedPositionWhileCalibrating = null;

  boolean disabledForDebugging = true;



  public ClimbElevationSubsystem() { //The constructor
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
    boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 17, "Climber") || shouldMakeAllCANDevices) {
      this.motor = new CANSparkMaxSendable(17, MotorType.kBrushless);
      addChild("motor", this.motor);
    }
    this.limitSwitch = new DigitalInput(8);
    
    if (motor != null) {
      MotorSetup motorSetup = new MotorSetup().setCoast(false).setCurrentLimit(80).setInverted(true);
      motorSetup.apply(motor);
      motorEncoder = motor.getEncoder();
      motorEncoder.setPositionConversionFactor(positionConverionFactor);
      motorEncoder.setVelocityConversionFactor(velocityConverionFactor);

      pid = motor.getPIDController();
      pid.setP(kP); // 
      pid.setI(kI); // 
      pid.setD(kD); // 
      pid.setFF(kFF); //
      pid.setOutputRange(negOutputLimit, posOutputLimit);
    }
  }

  @Override
  public void periodic() {
    // only do something if we actually have a motor
    if (motor != null && !disabledForDebugging) { 
      if (motorEncoder != null) { // if there is an encoder, display these
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) { 
            // If the robot is running, and the encoder is "not calibrated," run motor very slowly towards the switch
            motor.set(-.1);
            if (calibrationTimer == null) {
              // we need to calibrate and we have no timer. make one and start it
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              // we have a timer, has the motor had power long enough to spin up
              if (calibrationTimer.get() > 0.5) {
                // motor should be moving if limit switch not pressed
                if (Math.abs(motorEncoder.getVelocity()) < 2000) {
                  // limit switch pressed, stop the motor, set encoder position to 0, and set calibration to true
                  encoderCalibrated = true;
                  setPower(0.0);
                  motorEncoder.setPosition(-0.5);  // leave a fudge factor

                  // If there was a requested position while we were calibrating, go there
                  if (requestedPositionWhileCalibrating != null) {
                    setPosition(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }
                }
              }
            }
          } else {
            if (requestedPower != null) {
              if (! disabledForDebugging) {
                motor.set(requestedPower);
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
    position = MathUtil.clamp(position, 0, 11);
    SmartDashboard.putNumber(name + ".requestedPosition", position);
    requestedPosition = position;
    requestedPower = null;
    if (encoderCalibrated) {
      if (!disabledForDebugging) {
        pid.setReference(position, ControlType.kPosition);
      }
    } else {
      requestedPositionWhileCalibrating = position;
    }
  }
  //returns limit switch status
  public boolean isLimitSwitchPressed(){
    return limitSwitch.get();
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
  public void setPower(double power) {
    requestedPower = power;
  }

  @Override
  public void updateTelemetry() {
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);
    SmartDashboard.putBoolean(name + ".Limit status", isLimitSwitchPressed());

    if (motor != null) { 
      SmartDashboard.putNumber(name + ".current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

      if (motorEncoder != null) { // if there is an encoder, display these
        double velocity = motorEncoder.getVelocity();
        double position = motorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".velocity", velocity);
        SmartDashboard.putNumber(name + ".position", position);
      }
    }
  }

}
