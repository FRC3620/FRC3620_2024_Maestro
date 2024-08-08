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
    private double kSpinP = 0.215; 
    private double kSpinI = 0.00;
    private double kSpinD = 0.00; //0.01

    public PIDController turnToheadingPID;
    private double TurnkSpinP = 0.50; 
    private double TurnkSpinI = 0.00;
    private double TurnkSpinD = 0.00;

    SwerveSubsystem drivebase;
    Double headingSetpoint = null;

    public SuperSwerveController(SwerveSubsystem drivebase) {
        headingPID = new PIDController(kSpinP, kSpinI, kSpinD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI); // sets a circular range instead of a linear one.
        SmartDashboard.putData("Heading PID", headingPID);

        turnToheadingPID = new PIDController(TurnkSpinP, TurnkSpinI, TurnkSpinD);
        turnToheadingPID.enableContinuousInput(-Math.PI, Math.PI); // sets a circular range instead of a linear one.
        SmartDashboard.putData("Turn To Heading PID", turnToheadingPID);
    
        this.drivebase = drivebase;
        headingSetpoint = drivebase.getPose().getRotation().getDegrees();
    }

    Timer turnTimer = new Timer();
    //SmartDashboard.putNumber("turnTimer", turnTimer);

    public void turnTo(SwerveSubsystem swerve, double targetHeading) {

        double targetOmega = 0;
        if (headingSetpoint != null) {
            //targetOmega = headingPID.calculate(drivebase.getHeading().getDegrees(), targetHeading);
            targetOmega = turnToheadingPID.calculate(drivebase.getPose().getRotation().getRadians(), Math.toRadians(targetHeading));
            //targetOmega = MathUtil.clamp(targetOmega, -4, 4);
            headingSetpoint = targetHeading;
        }
        /*if(Math.abs(targetOmega) < 0.16) {
            targetOmega = 0;
        }*/
        swerve.drive(new Translation2d(0, 0), targetOmega*swerve.getMaximumAngularVelocity(), true);

        SmartDashboard.putNumber("SuperSwerve.targetOmega", targetOmega);
        if (headingSetpoint != null) {
            SmartDashboard.putNumber("SuperSwerve.headingSetpoint", headingSetpoint);
        }
        //SmartDashboard.putNumber("drivebase.getHeading", drivebase.getHeading().getDegrees());

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