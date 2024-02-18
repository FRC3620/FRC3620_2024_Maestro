// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeRollersMechanism {
  /** Creates a new IntakeRollersMechanism. */
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;
  DigitalInput gamePieceDetector;

  final String name = "intake.rollers";

  boolean disabledForDebugging = false;

  public IntakeRollersMechanism(CANSparkMaxSendable motor, DigitalInput gamePieceDetector) {
    this.motor = motor;
    this.gamePieceDetector = gamePieceDetector;
    if (motor != null) {
      this.encoder = motor.getEncoder();
      encoder.setPositionConversionFactor(1);
      encoder.setVelocityConversionFactor(1);
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(name + ".game_piece_detected", gamePieceDetected());

    if (motor != null) {
      if (RobotContainer.powerDistribution != null) {
        SmartDashboard.putNumber(name + ".motor_input_current", RobotContainer.powerDistribution.getCurrent(motor.getDeviceId()));
      }
      SmartDashboard.putNumber(name + ".motor_current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());
      SmartDashboard.putNumber(name + ".speed", encoder.getVelocity());
    }
  }

  public void setPower(double speed) {
    if (motor != null) {
      if (!disabledForDebugging) {
        motor.set(speed);
      }
    }
  }

  public boolean gamePieceDetected() {
    if (gamePieceDetector.get()) {
      return false;
    } else {
      return true;
    }
  }

  public double getVelocity() {
    if (encoder == null)
      return 0;
    return encoder.getVelocity();
  }
}