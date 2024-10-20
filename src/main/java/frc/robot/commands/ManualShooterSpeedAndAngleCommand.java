// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterElevationSubsystem;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;

/* This command will read inputs from ShuffleBoard and set the shooter speed and angle appropriately. The command will be initiated/ended from the Dashboard  */

public class ManualShooterSpeedAndAngleCommand extends Command {

  ShooterWheelsAndAmpBarSubsystem shooterWheelAndAmpBarSubsystem = RobotContainer.shooterWheelsAndAmpBarSubsystem;
  ShooterElevationSubsystem shooterElevationSubsystem = RobotContainer.shooterElevationSubsystem;

  /** Creates a new ManualShooterSpeedAndAngleCommand. */
  public ManualShooterSpeedAndAngleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterWheelAndAmpBarSubsystem, shooterElevationSubsystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // We need to read the data from SmartDashboard before we set the intiial Speed and Angle

    ShooterSpeedAndAngle speedAndAngle = new ShooterSpeedAndAngle(SmartDashboard.getNumber("shooter.manual.speed", 0), 
                                      SmartDashboard.getNumber("shooter.manual.angle", 0));

    shooterWheelAndAmpBarSubsystem.setRequestedWheelSpeed(speedAndAngle);
    shooterElevationSubsystem.setElevationPosition(speedAndAngle);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        // We need to read the data from SmartDashboard before we set the intiial Speed and Angle

    ShooterSpeedAndAngle speedAndAngle = new ShooterSpeedAndAngle(SmartDashboard.getNumber("shooter.manual.speed", 0), 
                                      SmartDashboard.getNumber("shooter.manual.angle", 0));

    shooterWheelAndAmpBarSubsystem.setRequestedWheelSpeed(speedAndAngle);
    shooterElevationSubsystem.setElevationPosition(speedAndAngle);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooterWheelAndAmpBarSubsystem.setRequestedWheelSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
