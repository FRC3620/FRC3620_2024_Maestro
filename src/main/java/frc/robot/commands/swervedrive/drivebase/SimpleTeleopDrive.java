// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SimpleTeleopDrive extends Command {
  /** Creates a new SimpleTeleopDrive. */

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;
  private final BooleanSupplier driveMode;

  public SimpleTeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode) {

    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xJoy = vX.getAsDouble();
    double yJoy = vY.getAsDouble();
    double oJoy = omega.getAsDouble();

    double xVelocity = Math.pow(xJoy, 3);
    double yVelocity = Math.pow(yJoy, 3);
    double oVelocity = Math.pow(oJoy, 3);

    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
      oVelocity * swerve.getMaximumAngularVelocity(), driveMode.getAsBoolean());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
