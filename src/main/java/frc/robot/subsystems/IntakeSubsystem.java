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

public class IntakeSubsystem extends SubsystemBase implements HasTelemetry {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public IntakeShoulderMechanism intakeShoulderMechanism;

  public CANSparkMaxSendable shoulder;

  public CANSparkMaxSendable rollers;
  public SparkLimitSwitch gamePieceObtained;
  double temporaryPosition;

  public IntakeSubsystem() {
    setupMotors();
    intakeShoulderMechanism = new IntakeShoulderMechanism(shoulder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeShoulderMechanism.periodic();
  }

  public void setManualShoulderPower(Double power) {
    intakeShoulderMechanism.setManualPower(power);
  }

  public void setShoulderPosition(Double angle) {
    intakeShoulderMechanism.setPositionSetpoint(angle);
  }

  public Double getRequestedShoulderPosition() {
    return intakeShoulderMechanism.getPositionSetpoint();
  }

  public double getActualShoulderElevation() {
    return intakeShoulderMechanism.getActualPosition();
  }

  public void recalibrataePitch(boolean forward) {
    recalibrataePitch(forward);
  }

  public SparkLimitSwitch getLimitSwitch() {
    return gamePieceObtained;
  }

  public final static int MOTORID_INTAKE_ROLLERS = 9;
  public final static int MOTORID_INTAKE_EXTEND = 10;
  public final static int MOTORID_INTAKE_EXTEND2 = 11;
  public final static int MOTORID_INTAKE_WRIST = 12;
  public final static int MOTORID_INTAKE_ELEVATION = 13;

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
    boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_ELEVATION, "Intake Elevation")
        || shouldMakeAllCANDevices) {
      shoulder = new CANSparkMaxSendable(MOTORID_INTAKE_ELEVATION, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(40).setCoast(false);
      motorSetup.apply(shoulder);
      addChild("elevation", shoulder);
    }
  }

  public void setLocation(IntakeLocation intakeLocation) {
    // logger.info ("Setting intake to {}", intakeLocation);
    intakeShoulderMechanism.setLocation(intakeLocation);

  }

  @Override
  public void updateTelemetry() {
    intakeShoulderMechanism.updateTelemetry();
  }
}

// :D