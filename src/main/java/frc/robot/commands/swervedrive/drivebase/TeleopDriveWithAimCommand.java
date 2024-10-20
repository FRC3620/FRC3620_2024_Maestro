// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.SuperSwerveController;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.usfirst.frc3620.misc.Utilities;

import com.ctre.phoenix.Util;

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
      BooleanSupplier driveMode, VisionSubsystem vision, SwerveController YAGSLController) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = YAGSLController;
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
    double xJoy = vX.getAsDouble();
    double yJoy = vY.getAsDouble();
    double aJoy = omega.getAsDouble();

    //double xVelocity = xJoy * xJoy * xJoy;
    //double yVelocity = yJoy * yJoy * yJoy;
    //double angVelocity = aJoy * aJoy * aJoy;

    double xVelocity = xJoy;
    double yVelocity = yJoy;
    double angVelocity = aJoy;
    

    var color = DriverStation.getAlliance();
    if(color.isPresent()){
          if(color.get()==Alliance.Red){
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
          }
        }

    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);

    // TODO: Update SwerveSubsystem to include boolean "AreWeAiming"
    // TODO: Update new teleopDriveWithAim command to check "areweaiming" and "do I
    // see the right target?"
    
    
    if(swerve.getAreWeAiming()){
      //grabs heading from vision subsystem

      // Turn Off Heading Correction
      //swerve.setHeadingCorrection(false);

      Double headingToTag = vision.getCamYawToSpeaker();
      //Note: headingToTag is a measurement of the yaw from the robot's perspective. 
      //we need to turn this into field orientation later
      
      if (headingToTag == null) {
        SmartDashboard.putString("aimDrive.doIHaveTarget", "No");
        angVelocity = 0;

      } else {
        //current heading

        // TODO: Currently, this points the FRONT of the robot toward the apriltag.
        // We'll need to modify this to point the rear of the robot to the AprilTag
        // when we get JoeHann back.
        
        double currentPosRotation = swerve.getPose().getRotation().getDegrees();
        double targetHeading =  Utilities.normalizeAngle(headingToTag);
        //calculates angVelocity
        angVelocity = controller.headingCalculate(Units.degreesToRadians(currentPosRotation),
                                                  Units.degreesToRadians(targetHeading));

        angVelocity = MathUtil.clamp(angVelocity, -swerve.getMaximumAngularVelocity(), swerve.getMaximumAngularVelocity() );
                                        
                                          
        
        //angVelocity = aimController.calculate(currentPosRotation,targetHeading);
        //Prints on dashboard
        SmartDashboard.putString("aimDrive.doIHaveTarget", "Yes");
        
        SmartDashboard.putNumber("aimDrive.currentRobotRotation", currentPosRotation);
        SmartDashboard.putNumber("aimDrive.headingToTag", headingToTag);
        SmartDashboard.putNumber("aimDrive.targetHeading", targetHeading);
        SmartDashboard.putNumber("aimDrive.lastAngVelocity", angVelocity);
        //SmartDashboard.putNumber("aimDrive.headingError", controller.headingPID.getPositionError());
      }
    
    
      
    } else {
      angVelocity = angVelocity*swerve.getMaximumAngularVelocity();
    }
    SmartDashboard.putNumber("aimDrive.AngVelocity", angVelocity);
    // Drive using raw values.

    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
        angVelocity,
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
