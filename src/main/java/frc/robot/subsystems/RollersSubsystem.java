package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class RollersSubsystem extends SubsystemBase implements HasTelemetry {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public IntakeRollersMechanism intakeRollerMechanism;

  public CANSparkMaxSendable rollers;
  public DigitalInput gamePieceObtained;

  public RollersSubsystem() {
    setupMotors();
    intakeRollerMechanism = new IntakeRollersMechanism(rollers, gamePieceObtained);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeRollerMechanism.periodic();
  }

  public void setRollerPower(double rollerSpeed) {
    intakeRollerMechanism.setPower(rollerSpeed);
  }

  public double getRollerVelocity() {
    return intakeRollerMechanism.getVelocity();
  }

  public boolean gamePieceDetected() {
    return intakeRollerMechanism.gamePieceDetected();
  }

  public final static int MOTORID_INTAKE_ROLLERS = 9;

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
    boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_ROLLERS, "Rollers")
        || shouldMakeAllCANDevices) {
      rollers = new CANSparkMaxSendable(MOTORID_INTAKE_ROLLERS, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(40).setCoast(true);
      motorSetup.apply(rollers);
      addChild("roller", rollers);
    }

    gamePieceObtained = new DigitalInput(9);
  }

  @Override
  public void updateTelemetry() {
    intakeRollerMechanism.updateTelemetry();
  }
}
