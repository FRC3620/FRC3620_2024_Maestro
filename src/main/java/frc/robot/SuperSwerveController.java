// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/** Add your docs here. */
public class SuperSwerveController {
    double headingSetpoint;
    PIDController headingPID;
    private double kSpinP = 0.0055;
	private double kSpinI = 0.00000;
	private double kSpinD = 0.00;
    SwerveSubsystem drivebase;
    

    public SuperSwerveController(SwerveSubsystem drivebase) {        
        headingPID = new PIDController(kSpinP, kSpinI, kSpinD);
        headingPID.enableContinuousInput(-180, 180); //sets a circular range instead of a linear one. 
        headingPID.setTolerance(5);
        this.drivebase = drivebase;
    } 

    Timer turnTimer = null;

    public void doTeleop(SwerveSubsystem swerve, double rX, double rY, double rOmega) {
        double targetOmega = 0;

        if (Math.abs(rOmega) < 0.1) {  //TODO: Change minimum Omega value to CONSTANT
            // There is not enough rotational input -- Keep the previous heading value
            //get heading
            if (turnTimer == null) {
                turnTimer = new Timer();
                turnTimer.reset();
                turnTimer.start();
            } else if (turnTimer.get() > 0.5) {
                headingSetpoint = drivebase.getHeading().getDegrees();
                turnTimer.stop();
                turnTimer.reset();
                targetOmega = headingPID.calculate(drivebase.getHeading().getDegrees(), headingSetpoint);
            } else {
                targetOmega = 0;
            }
        } else {
            // The driver is providing rotational input. Set Omega according to driver input
            // and store current heading
            turnTimer = null;
            targetOmega = rOmega;
        }
        
        targetOmega = MathUtil.clamp(targetOmega, -1, 1);

        swerve.drive(new Translation2d(rX, rY), targetOmega, true);
        SmartDashboard.putNumber("SuperSwerve.targetOmega", targetOmega);
        SmartDashboard.putNumber("SuperSwerve.rOmega", rOmega);
        SmartDashboard.putNumber("SuperSwerve.headingSetpoint", headingSetpoint);

    }
}
