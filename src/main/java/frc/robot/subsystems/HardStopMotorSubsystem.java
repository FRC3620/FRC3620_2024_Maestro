// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HardStopMotorSubsystem extends SubsystemBase {
  boolean encoderCalibrated = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;

  final String name = "Motor";

  // CHECK THIS
  SparkPIDController PID = null;

  Double requestedPositionWhileCalibrating = null;
  double requestedPosition = 0;

  /** Creates a new HardStopMotorSubsystem. */
  public HardStopMotorSubsystem() {
    this.motor = new CANSparkMaxSendable(11, MotorType.kBrushless);
    if (motor != null) {
      this.encoder = motor.getEncoder();

      PID = motor.getPIDController();

      // set up PID for turretPID here
      PID.setP(0.2); // 0.1
      PID.setI(0.0); // 0.0
      PID.setD(0); // 10
      PID.setFF(0.0); // 0.0

      PID.setOutputRange(-0.2, 0.2);
    }

    if (encoder != null) {
      encoder.setPositionConversionFactor(15 / 30.4);
      // encoder.setVelocityConversionFactor(1);
    }
  }

  public void moveSlowly(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);

    if (motor != null) {
      SmartDashboard.putNumber(name + ".current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

      if (encoder != null) {
        double spinSpeed = encoder.getVelocity();
        double spinPosition = encoder.getPosition();
        SmartDashboard.putNumber(name + ".speed", spinSpeed);
        SmartDashboard.putNumber(name + ".position", spinPosition);

        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) {
            moveSlowly(0.03);

            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.5) {
                if (Math.abs(spinSpeed) < 2) {
                  encoderCalibrated = true;
                  moveSlowly(0.0);
                  encoder.setPosition(0.0);
                  if (requestedPositionWhileCalibrating != null) {
                    setExtension(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
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

  public void setExtension(double length) {
    length = MathUtil.clamp(length, 0, 45);
    SmartDashboard.putNumber(name + ".requestedLength", length);
    requestedPosition = length;
    if (encoderCalibrated) {
      PID.setReference(length, ControlType.kPosition);
    } else {
      requestedPositionWhileCalibrating = length;
    }
  }

}
