// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class SuperSwerveController {
    public PIDController headingPID;
    private double kSpinP = 0.0325; 
    private double kSpinI = 0.00;
    private double kSpinD = 0.00; //0.01
    SwerveSubsystem drivebase;
    Double headingSetpoint = null;

    public SuperSwerveController(SwerveSubsystem drivebase) {
        headingPID = new PIDController(kSpinP, kSpinI, kSpinD);
        headingPID.enableContinuousInput(-180, 180); // sets a circular range instead of a linear one.
        headingPID.setTolerance(2.5);
        this.drivebase = drivebase;
        headingSetpoint = drivebase.getHeading().getDegrees();
    }

    Timer turnTimer = new Timer();
    //SmartDashboard.putNumber("turnTimer", turnTimer);

    public void doTeleop(SwerveSubsystem swerve, double rX, double rY, double rOmega) {

            double targetOmega = 0;

            if (Math.abs(rOmega) < 0.1) { // TODO: Change minimum Omega value to CONSTANT
                // There is not enough rotational input -- Keep the previous heading value
                // get heading  
                if (turnTimer.get() < 0.05) {
                    //headingSetpoint = drivebase.getHeading().getDegrees();
                    targetOmega = headingPID.calculate(drivebase.getHeading().getDegrees(), headingSetpoint);
                } else {
                    targetOmega = 0;
                }
            } else {
                turnTimer.restart();
                headingSetpoint = drivebase.getHeading().getDegrees();
                targetOmega = rOmega;
                // The driver is providing rotational input. Set Omega according to driver input
                // and store current heading
            }

            //targetOmega = MathUtil.clamp(targetOmega, -1, 1);
    
            swerve.drive(new Translation2d(
                            Math.pow(rX, 3) * swerve.getMaximumVelocity(), 
                            Math.pow(rY, 3) * swerve.getMaximumVelocity()), 
                            Math.pow(targetOmega, 3) * swerve.getMaximumAngularVelocity()                                                                               , 
                            true);
            SmartDashboard.putNumber("SuperSwerve.targetOmega", targetOmega);
            SmartDashboard.putNumber("SuperSwerve.rOmega", rOmega);
            SmartDashboard.putNumber("SuperSwerve.calculatedX", Math.pow(rX, 3) * swerve.getMaximumVelocity());
            SmartDashboard.putNumber("SuperSwerve.calculatedY", Math.pow(rY, 3) * swerve.getMaximumVelocity());
            SmartDashboard.putNumber("SuperSwerve.calculatedOmega", Math.pow(rOmega, 3) * swerve.getMaximumAngularVelocity());
            if (headingSetpoint != null) {
                SmartDashboard.putNumber("SuperSwerve.headingSetpoint", headingSetpoint);
            }
            SmartDashboard.putNumber("SuperSwerve.getHeading", drivebase.getHeading().getDegrees());
    }

    public void turnTo(SwerveSubsystem swerve, double targetHeading) {
        double targetOmega = 0;
        if (headingSetpoint != null) {
            targetOmega = headingPID.calculate(drivebase.getHeading().getDegrees(), targetHeading);
            targetOmega = MathUtil.clamp(targetOmega, -4, 4);
            headingSetpoint = targetHeading;
        }
        if(Math.abs(targetOmega) < 0.16) {
            targetOmega = 0;
        }
        swerve.drive(new Translation2d(0, 0), targetOmega, true);

        SmartDashboard.putNumber("SuperSwerve.targetOmega", targetOmega);
        if (headingSetpoint != null) {
            SmartDashboard.putNumber("SuperSwerve.headingSetpoint", headingSetpoint);
        }
        SmartDashboard.putNumber("drivebase.getHeading", drivebase.getHeading().getDegrees());

    }

    public double headingCalculate(double currentHeadingAngleRadians, double targetHeadingAngleRadians)
  {

    // Display PID Inputs
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putNumber("SwerveController.kP", headingPID.getP());
      SmartDashboard.putNumber("SwerveController.kI", headingPID.getI());
      SmartDashboard.putNumber("SwerveController.kD", headingPID.getD());
      SmartDashboard.putNumber("SwerveController.PositionError", headingPID.getPositionError());
      SmartDashboard.putNumber("SwerveController.currentHeadingAngleRadians", currentHeadingAngleRadians);
      SmartDashboard.putNumber("SwerveController.targetHeadingAngleRadians", targetHeadingAngleRadians);
      SmartDashboard.putNumber("SwerveController.currentHeadingAngleDegrees", Units.radiansToDegrees(currentHeadingAngleRadians));
      SmartDashboard.putNumber("SwerveController.targetHeadingAngleDegrees", Units.radiansToDegrees(targetHeadingAngleRadians));
    }
    return headingPID.calculate(currentHeadingAngleRadians, targetHeadingAngleRadians)                                                                                                                                                  * 50;
  }
}


//:)