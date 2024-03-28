// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterElevationSubsystem;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourcePickupCommand extends SequentialCommandGroup {
  ShooterWheelsAndAmpBarSubsystem shooterWheels = RobotContainer.shooterWheelsAndAmpBarSubsystem;

  /** Creates a new SourcePickupCommand. */
  public SourcePickupCommand() {
    addCommands(
        new InnerSourcePickupCommand(),
        new InstantCommand(() -> shooterWheels.setRequestedWheelSpeed(0), shooterWheels)
      );
  }

  /**
   * InnerSourcePickupCommand
   */
  public class InnerSourcePickupCommand extends ParallelDeadlineGroup {
    /** Creates a new SourcePickupCommand. */
    public InnerSourcePickupCommand() {
      // addCommands().
      super(new RunIndexerUntilGamePieceDetectedCommand(() -> -0.8));
      addCommands(
          new SetShooterSpeedAndAngleCommand(ShooterSpeedAndAngle.sourcePickup)
        );
    }
  }
}
