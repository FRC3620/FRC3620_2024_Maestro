package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class RunIndexerCommand extends Command {
  IndexerSubsystem indexerSubsystem = RobotContainer.indexerSubsystem;

  Logger logger = EventLogging.getLogger(getClass());

  DoubleSupplier savedPowerSupplier;

  public RunIndexerCommand(DoubleSupplier powerSupplier) {
    savedPowerSupplier = requireNonNullParam(powerSupplier, "powerSupplier", "<init>");
    addRequirements(indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = savedPowerSupplier.getAsDouble();
    indexerSubsystem.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  boolean gamePieceDetected() {
    return indexerSubsystem.gamePieceDetected();
  }

  void setLimitSwitchEnabled(boolean enabled) {
    indexerSubsystem.setLimitSwitchEnabled(enabled);
  }

}
