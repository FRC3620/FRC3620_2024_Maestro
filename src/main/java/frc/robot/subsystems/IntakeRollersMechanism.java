// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class IntakeRollersMechanism implements HasTelemetry {
  /** Creates a new IntakeRollersMechanism. */
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;
  SparkLimitSwitch gamePieceDetector;

  final String name = "intake.rollers";

  boolean disabledForDebugging = false;

  public IntakeRollersMechanism(CANSparkMaxSendable motor) {
    this.motor = motor;
    if (motor != null) {
      this.encoder = motor.getEncoder();
      encoder.setPositionConversionFactor(1);
      encoder.setVelocityConversionFactor(1);

      gamePieceDetector = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(name + "limit_switch_ispressed", gamePieceDetector.isPressed());
    SmartDashboard.putBoolean(name + "game_peice_detected", gamePieceDetected());

    if (motor != null) {
      SmartDashboard.putNumber(name + ".motor_current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());

      if (encoder != null) {
        double elevateSpeed = encoder.getVelocity();
        SmartDashboard.putNumber(name + ".speed", elevateSpeed);
        // SmartDashboard.putNumber(name + ".velocityConversionFactor",
        // encoder.getVelocityConversionFactor());

        /*
         * if(Robot.getCurrentRobotMode() == RobotMode.TELEOP ||
         * Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
         * if (!encoderIsValid) {
         * moveClawCannon(0.015);
         * 
         * if (calibrationTimer == null) {
         * calibrationTimer = new Timer();
         * calibrationTimer.reset();
         * calibrationTimer.start();
         * } else {
         * if (calibrationTimer.get() > 0.75){
         * if (Math.abs(elevateSpeed) < 15) {
         * encoderIsValid = true;
         * moveClawCannon(0.0);
         * encoder.setPosition(65);
         * 
         * if (requestedPositionWhileCalibrating != null) {
         * setPosition(requestedPositionWhileCalibrating);
         * requestedPositionWhileCalibrating = null;
         * }
         * }
         * }
         * }
         * }
         * } else {
         * calibrationTimer = null; // start over!
         * }
         */
      }
    }
  }

  /**
   * Sets the cannon to extend the arm to 'length' inches. Increasing
   * length is a longer arm.
   * "Extend" motor.
   */

  public void setPower(double speed) {
    if (motor != null) {
      if (!disabledForDebugging) {
        motor.set(speed);
      }
    }
  }


  public boolean gamePieceDetected() {
    if(gamePieceDetector != null){
      if (gamePieceDetector.isPressed()) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

  public double getVelocity() {
    if (encoder == null)
      return 0;
    return encoder.getVelocity();
  }

  @Override
  public void updateTelemetry() {
    SmartDashboard.putBoolean(name + ".game_piece_detected", gamePieceDetected());
    if (motor != null) {
      if (RobotContainer.powerDistribution != null) {
        //SmartDashboard.putNumber(name + ".motor_input_current", RobotContainer.powerDistribution.getCurrent(motor.getDeviceId()));
      }
      SmartDashboard.putNumber(name + ".motor_current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());
      SmartDashboard.putNumber(name + ".speed", encoder.getVelocity());
    }
  }
  
}