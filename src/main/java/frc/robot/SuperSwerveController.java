// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/** Add your docs here. */
public class SuperSwerveController {
    double headingSetpoint;
    PIDController headingPID;
    private double kSpinP = 0.0055;
	private double kSpinI = 0.00000;
	private double kSpinD = 0.00;
    SwerveSubsystem drivebase;
    double targetOmega;

    public SuperSwerveController(SwerveSubsystem drivebase) {        
        headingPID = new PIDController(kSpinP, kSpinI, kSpinD);
        headingPID.enableContinuousInput(-180, 180); //sets a circular range instead of a linear one. 
        headingPID.setTolerance(3);
        this.drivebase = drivebase;
    } 

    public void doTeleop(SwerveSubsystem swerve, double rX, double rY, double rOmega) {
        
        if (Math.abs(rOmega) < 0.075) {  //TODO: Change minimum Omega value to CONSTANT
            // There is not enough rotational input -- Keep the previous heading value
            //get heading
            targetOmega = headingPID.calculate(drivebase.getHeading().getDegrees(), headingSetpoint);
        } else {
            // The driver is providing rotational input. Set Omega accordin to driver input
            // and store current heading
            targetOmega = rOmega;
            headingSetpoint = drivebase.getHeading().getDegrees();
        }
        
        targetOmega = MathUtil.clamp(targetOmega, -1, 1);

        swerve.drive(new Translation2d(rX, rY), targetOmega, true);
        SmartDashboard.putNumber("SuperSwerve.targetOmega", targetOmega);
        SmartDashboard.putNumber("SuperSwerve.rOmega", rOmega);
        SmartDashboard.putNumber("SuperSwerve.headingSetpoint", headingSetpoint);

    }
}
