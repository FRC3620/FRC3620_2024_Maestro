// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeLocation;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundToHomeCommand extends SequentialCommandGroup {
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
  /** Creates a new GroundPickupCommand. */
  public GroundToHomeCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntakeLocationCommand(IntakeLocation.midGroundPositionForHome),
      new WaitUntilCommand(() -> intakeSubsystem.getActualShoulderElevation() > 12),
      new SetIntakeLocationCommand(IntakeLocation.homePosition),
      new WaitUntilCommand(() -> intakeSubsystem.getActualExtendPosition() < 1.5 && intakeSubsystem.getActualShoulderElevation() < 5),
      new SetIntakeLocationCommand(IntakeLocation.parkPosition)
    );
  }
}
