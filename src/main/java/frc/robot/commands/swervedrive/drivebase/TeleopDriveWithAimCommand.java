// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperSwerveController;
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
  private final SuperSwerveController controller;

  private final VisionSubsystem vision;
  private boolean areweaiming;

  //PIDController aimController;
  //final double kSpinP = 0.035; 
  //final double kSpinI = 0.00;
  //final double kSpinD = 0.00; //0.01

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDriveWithAimCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
      BooleanSupplier driveMode, VisionSubsystem vision, SuperSwerveController superDupSwerveController) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = superDupSwerveController;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    // Define aimController
    //aimController = new PIDController(kSpinP, kSpinI, kSpinD);
    //aimController.enableContinuousInput(-180.0, 180.0);
    //aimController.setTolerance(1.0);

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
    
    
    if(swerve.getAreWeAiming())
  {
      //grabs heading from vision subsystem

      // Turn Off Heading Correction
      //swerve.setHeadingCorrection(false);

      vision.camYawToSpeaker();
      Double headingToTag = vision.getCamYawToSpeaker();
      //Note: headingToTag is a measurement of the yaw from the robot's perspective. 
      //we need to turn this into field orientation later
      
      if (headingToTag == null) {
        SmartDashboard.putString("aimDrive.doIHaveTarget", "No");
        angVelocity = 0;

      } else {
        //current heading

        double currentPosRotation = swerve.getHeading().getDegrees();
        double targetHeading = currentPosRotation + headingToTag+6.5;
        //calculates angVelocity
        angVelocity = controller.headingCalculate(Units.degreesToRadians(currentPosRotation),
                                                  Units.degreesToRadians(targetHeading));
                                          
        
        //angVelocity = aimController.calculate(currentPosRotation,targetHeading);
        //Prints on dashboard
        SmartDashboard.putString("aimDrive.doIHaveTarget", "Yes");
        
        SmartDashboard.putNumber("aimDrive.currentRobotRotation", currentPosRotation);
        SmartDashboard.putNumber("aimDrive.headingToTag", headingToTag);
        SmartDashboard.putNumber("aimDrive.targetHeading", targetHeading);
        SmartDashboard.putNumber("aimDrive.lastAngVelocity", angVelocity);
        SmartDashboard.putNumber("aimDrive.headingError", controller.headingPID.getPositionError());
      }
    
    
      
    } else {
      //swerve.setHeadingCorrection(true);
    }
    SmartDashboard.putNumber("aimDrive.AngVelocity", angVelocity);
    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
        angVelocity * 5.1,
        driveMode.getAsBoolean());
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
