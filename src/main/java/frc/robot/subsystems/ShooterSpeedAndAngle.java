// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSpeedAndAngle extends SubsystemBase {
  double speed;
  double position;

  public static ShooterSpeedAndAngle testshooter1 = new ShooterSpeedAndAngle(10, 40);
  public static ShooterSpeedAndAngle testshooter2 = new ShooterSpeedAndAngle(20, 60);
  public static ShooterSpeedAndAngle shootingPosition = new ShooterSpeedAndAngle(5000, 37);
  public static ShooterSpeedAndAngle subWoofShot = new ShooterSpeedAndAngle(5000, 66.5);
  public static ShooterSpeedAndAngle shooterHome = new ShooterSpeedAndAngle(0, 0);
  public static ShooterSpeedAndAngle ejectAllShooter = new ShooterSpeedAndAngle(20, 30);
  public static ShooterSpeedAndAngle disabledUp = new ShooterSpeedAndAngle(0, 68);
  public static ShooterSpeedAndAngle ampShot = new ShooterSpeedAndAngle(1500, 52);
  public static ShooterSpeedAndAngle sourcePickup = new ShooterSpeedAndAngle(-2000, 60);

  /** Creates a new ShooterSpeedAndAngle. */
  public ShooterSpeedAndAngle(double speed, double position) {
    this.speed = speed;
    this.position = position;
  }

  public double getPosition() {
    return position;
  }

  public double getSpeed() {
    return speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
