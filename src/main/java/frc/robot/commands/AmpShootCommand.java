// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeLocation;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSpeedAndAngle;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShootCommand extends SequentialCommandGroup {
  ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
  ShooterSpeedAndAngle shooterSpeedAndAngle= new ShooterSpeedAndAngle(10, 50);
  Timer timer= new Timer();
  /** Creates a new AmpShootCommand. */
  public AmpShootCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*addCommands(
      new SetIntakeLocationCommand(IntakeLocation.ampPosition)
    );*/
    addCommands(
   new InstantCommand(()->shooterSubsystem.setAmpBarPosition(7)),
   new WaitUntilCommand(()->shooterSubsystem.getAmpBarPosition()>6),
  new InstantCommand(()->shooterSubsystem.setSpeedAndAngle(shooterSpeedAndAngle)),
  new InstantCommand(()-> timer.restart()),
  new  WaitUntilCommand(()-> timer.get()==0.2),
  new InstantCommand(()-> shooterSubsystem.setAmpBarPosition(0))
    );
  }
}
