package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class RollersSubsystem extends SubsystemBase implements HasTelemetry {
  public final static int MOTORID_INTAKE_ROLLERS = 9;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public IntakeRollersMechanism intakeRollerMechanism;

  public CANSparkMaxSendable rollers;
  
  public RollersSubsystem() {
    setupMotors();
    intakeRollerMechanism = new IntakeRollersMechanism(rollers);
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

  public double getRollerPosition() {
    return intakeRollerMechanism.getPosition();
  }

  /*
   * this was replaced by moving the game detector to the indexer. 
   * keeping this here as a private to facilitate code changes in case
   * we need to move it back!
   */
  private boolean gamePieceDetected() {
    return intakeRollerMechanism.gamePieceDetected();
  }

  private SparkLimitSwitch getLimitSwitch() {
    return intakeRollerMechanism.gamePieceDetector;
  }

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
    boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_ROLLERS, "Rollers")
        || shouldMakeAllCANDevices) {
      rollers = new CANSparkMaxSendable(MOTORID_INTAKE_ROLLERS, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(40).setCoast(false);
      motorSetup.apply(rollers);
      addChild("roller", rollers);
    }
  }

  @Override
  public void updateTelemetry() {
    intakeRollerMechanism.updateTelemetry();
  }
}
