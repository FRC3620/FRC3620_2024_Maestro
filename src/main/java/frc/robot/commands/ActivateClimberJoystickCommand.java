// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbElevationSubsystem;
import frc.robot.subsystems.IntakeLocation;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem;
import frc.robot.subsystems.ShooterWheelsAndAmpBarSubsystem.AmpBarPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActivateClimberJoystickCommand extends SequentialCommandGroup {
  /** Creates a new ActivateCimberJoystickCommand. */
  public ActivateClimberJoystickCommand() {
    ClimbElevationSubsystem climbElevationSubsystem = RobotContainer.climbElevationSubsystem;
    ShooterWheelsAndAmpBarSubsystem ampBarSubsystem = RobotContainer.shooterWheelsAndAmpBarSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntakeLocationCommand(IntakeLocation.IntakeOut),
      new InstantCommand(() -> ampBarSubsystem.setAmpBarPosition(AmpBarPosition.UP)),
      new SetClimberPositionCommand(12.325),
      new WaitUntilCommand(() -> climbElevationSubsystem.getActualPosition() > 12),
      new SetClimberPowerPositionCommand()
    );
  }
}
