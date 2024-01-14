// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX motor1;
  TalonFX motor2;

  /** Creates a new PropellorSubsytem. */
  public ShooterSubsystem() {
    motor1 = new TalonFX(1);
    motor2 = new TalonFX(2);
  }

  public void spinMotor1(double power1) {
    motor1.set(power1);
    SmartDashboard.putNumber("motor1Actual.power", power1);
  }

  public void spinMotor2(double power2) {
    motor2.set(power2);
    SmartDashboard.putNumber("motor2Actual.power", power2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
