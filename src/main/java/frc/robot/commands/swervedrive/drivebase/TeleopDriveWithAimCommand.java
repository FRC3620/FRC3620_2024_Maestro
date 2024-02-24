// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDriveWithAimCommand extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;
  private final BooleanSupplier driveMode;
  private final SwerveController controller;

  private final VisionSubsystem vision;
  private boolean areweaiming;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDriveWithAimCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
      BooleanSupplier driveMode, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3);
    double yVelocity = Math.pow(vY.getAsDouble(), 3);
    double angVelocity = Math.pow(omega.getAsDouble(), 3);
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);

    // TODO: Update SwerveSubsystem to include boolean "AreWeAiming"
    // TODO: Update new teleopDriveWithAim command to check "areweaiming" and "do I
    // see the right target?"
    
    if(swerve.AreWeAiming(areweaiming) == true){
      //grabs heading from vision subsystem
      Double headingToTag = vision.camYawToSpeaker();

      if (headingToTag == null) {
        SmartDashboard.putString("doIHaveTarget", "No");

      } else {
        //current heading
        double currentPosRotation = swerve.getHeading().getDegrees();
        //calculates angVelocity
        double angularVelocity = controller.headingCalculate(currentPosRotation, headingToTag);
        //Prints on dashboard
        SmartDashboard.putString("doIHaveTarget", "Yes");
        SmartDashboard.putNumber("AngVelocity", angularVelocity);
      }
    } else {// else drive normally
      // Drive using raw values.
      swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
          angVelocity * controller.config.maxAngularVelocity,
          driveMode.getAsBoolean());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
