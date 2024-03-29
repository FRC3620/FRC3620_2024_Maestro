package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAnglePIDZapper extends InstantCommand {
  IntakeSubsystem subsystem = RobotContainer.intakeSubsystem;
  SparkPIDController pid;

  /** Creates a new ShooterAnglePIDZapper. */
  public IntakeAnglePIDZapper() {
    addRequirements(subsystem);
    if (subsystem.shoulder != null) {

      pid = subsystem.shoulder.getPIDController();

      SmartDashboard.putNumber("test.intake.angle.pid.actual.kP", pid.getP());
      SmartDashboard.putNumber("test.intake.angle.pid.actual.kI", pid.getI());
      SmartDashboard.putNumber("test.intake.angle.pid.requested.kP", pid.getP());
      SmartDashboard.putNumber("test.intake.angle.pid.requested.kI", pid.getI());
    }
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (subsystem.shoulder != null) {
    double kP = SmartDashboard.getNumber("test.intake.angle.pid.requested.kP", 0);
    double kI = SmartDashboard.getNumber("test.intake.angle.pid.requested.kI", 0);
    pid.setP(kP);
    pid.setI(kI);
    SmartDashboard.putNumber("test.intake.angle.pid.actual.kP", pid.getP());
    SmartDashboard.putNumber("test.intake.angle.pid.actual.kI", pid.getI());
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
