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
  public final static int MOTORID_INTAKE_ELEVATION = 13;

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

  public void setShoulderPosition(Double angle) {
    intakeShoulderMechanism.setPosition(angle);
  }

  public Double getRequestedShoulderPosition() {
    return intakeShoulderMechanism.getRequestedPosition();
  }

  public double getActualShoulderElevation() {
    return intakeShoulderMechanism.getActualPosition();
  }

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
    intakeShoulderMechanism.setPosition(intakeLocation.getShoulder());

  }

  @Override
  public void updateTelemetry() {
    intakeShoulderMechanism.updateTelemetry();
  }
}

// :D