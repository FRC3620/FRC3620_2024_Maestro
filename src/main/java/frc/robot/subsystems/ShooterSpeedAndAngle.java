// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSpeedAndAngle extends SubsystemBase {
  double speed;
  double position;

  public static ShooterSpeedAndAngle testshooter1 = new ShooterSpeedAndAngle(0, 0);
  public static ShooterSpeedAndAngle testshooter2 = new ShooterSpeedAndAngle(1, 1);
  
  /** Creates a new ShooterSpeedAndAngle. */
  public ShooterSpeedAndAngle(double speed, double position) {
    this.speed = speed;
    this.position = position;
  }

  public double getPosition() {
    return position;
  }

  public double getVelocity() {
    return speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
