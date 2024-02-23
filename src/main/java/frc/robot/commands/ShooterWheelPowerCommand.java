// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.Utilities.SlidingWindowStats;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterWheelPowerCommand extends Command {
  ShooterSubsystem subsystem = RobotContainer.shooterSubsystem;

  SlidingWindowStats topSpeed, bottomSpeed;

  /** Creates a new ShooterPowerCommand. */
  public ShooterWheelPowerCommand() {
    SmartDashboard.putNumber("shooter.wheels.power", 0);
    topSpeed = new SlidingWindowStats(100);
    bottomSpeed = new SlidingWindowStats(100);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    topSpeed.clear();
    bottomSpeed.clear();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = SmartDashboard.getNumber("shooter.wheels.power", 0);
    subsystem.setWheelPower(power);

    double s = subsystem.getTopMotorVelocity();
    topSpeed.addValue(s);
    s = subsystem.getBottomMotorVelocity();
    bottomSpeed.addValue(s);
    putStats("top", topSpeed);
    putStats("bottom", bottomSpeed);
  }

  void putStats(String s, SlidingWindowStats stats) {
    SmartDashboard.putNumber("intake." + s + ".slide.mean", stats.getMean());
    SmartDashboard.putNumber("intake." + s + ".slide.stdev", stats.getStdDev());
    SmartDashboard.putNumber("intake." + s + ".slide.flyers", stats.getFlyers());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setWheelPower(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
