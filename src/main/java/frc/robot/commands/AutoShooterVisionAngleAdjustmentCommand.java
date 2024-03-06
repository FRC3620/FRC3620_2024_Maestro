// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterCalcutlaiter;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoShooterVisionAngleAdjustmentCommand extends Command {
  private final VisionSubsystem vision;
  private final ShooterSubsystem shooter;
  boolean isFinished = false;

  /** Creates a new ShooterVisionAngleAdjustmentCommand. */
  public AutoShooterVisionAngleAdjustmentCommand(VisionSubsystem vision, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.shooter = shooter;

    addRequirements(shooter);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double distance = vision.camDistToSpeakerTag();
    if(distance == null){
      SmartDashboard.putString("shooterVision.doIHaveTarget", "no");
    }else{
      ShooterSpeedAndAngle angle= ShooterCalcutlaiter.CalculaiteAngleM(distance);
      shooter.setElevationPosition(angle.getPosition());
      SmartDashboard.putNumber("shooterVision.Angle", angle.getPosition());
      SmartDashboard.putString("shooterVision.doIHaveTarget", "yes");
      SmartDashboard.putNumber("shooterVision.Distance", distance);
      SmartDashboard.putNumber("shooterVision.DistanceFt", Units.metersToFeet(distance));
      SmartDashboard.putNumber("shooterVision.Yaw", vision.getCamYawToSpeaker());
    }

    if (shooter.getElevationPosition() > (ShooterCalcutlaiter.CalculaiteAngleM(distance).getPosition() - 1) && shooter.getElevationPosition() < (ShooterCalcutlaiter.CalculaiteAngleM(distance).getPosition() + 1)) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
