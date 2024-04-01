package frc.robot.commands;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SingCommand extends Command {
  FurElise _music = new FurElise();

  TalonFX talonFX = RobotContainer.shooterWheelsAndAmpBarSubsystem.topMotor;
  MusicTone musicTone = new MusicTone(0);

  public SingCommand() {
    addRequirements(RobotContainer.shooterWheelsAndAmpBarSubsystem);
  }

  @Override
  public void initialize() {
    _music.reset();
  }

  @Override
  public void execute() {
    int dt = 20; // 20ms per loop

    double freq = _music.GetMusicFrequency(dt);

    note(freq);
  }

  void note(double freq) {
    talonFX.setControl(musicTone.withAudioFrequency(freq));
  }

  @Override
  public void end(boolean interrupted) {
    note(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
