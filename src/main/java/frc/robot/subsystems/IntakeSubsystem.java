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
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase implements HasTelemetry {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public IntakeExtendMechanism intakeExtendMechanism;
  public IntakeShoulderMechanism intakeShoulderMechanism;
  public IntakeWristMechanism intakeWristMechanism;

  public CANSparkMaxSendable shoulder;

  public CANSparkMaxSendable extend;
  public DigitalInput extendHomeSwitch;

  public CANSparkMaxSendable extend2;

  public CANSparkMaxSendable wrist;
  public DigitalInput wristHomeSwitch;

  public CANSparkMaxSendable rollers;
  public SparkLimitSwitch gamePieceObtained;
  double temporaryPosition;

  public IntakeSubsystem() {
    setupMotors();
    intakeExtendMechanism = new IntakeExtendMechanism(extend, extend2);
    intakeShoulderMechanism = new IntakeShoulderMechanism(shoulder);
    intakeWristMechanism = new IntakeWristMechanism(wrist, wristHomeSwitch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeExtendMechanism.periodic();
    intakeShoulderMechanism.periodic();
    intakeWristMechanism.periodic();

    /*
    if (cantElevate()) {
      temporaryPosition = getActualShoulderElevation();
      // tell shoulder to stay where it is right now
      setShoulderPosition(temporaryPosition);
      // sets extend to requested position
      setExtendPosition(getRequestedExtendPosition());

    } else {
      // we can extend
      // tell shoulder to go to the position requested
      setShoulderPosition(getRequestedShoulderPosition());
    }

    if (cantHome()) {
      temporaryPosition = getActualExtendPosition();
      // tell shoulder to stay where it is right now
      setExtendPosition(temporaryPosition);
      // sets extend to requested position
      setShoulderPosition(getRequestedShoulderPosition());

    } else {
      // we can extend
      // tell shoulder to go to the position requested
      setExtendPosition(getRequestedExtendPosition());
    }
    */
  }

  boolean cantElevate() {
    double extendPosition;
    extendPosition = intakeExtendMechanism.getActualPosition();
    // change 1 to actual restraint
    if (extendPosition > 14) {
      return false;
    } else {
      return true;
    }
  }

  boolean cantHome() {
    double elevatePosition;
    elevatePosition = intakeExtendMechanism.getActualPosition();
    // change 1 to actual restraint
    if (2 > elevatePosition && elevatePosition > 1) {
      return false;
    } else {
      return true;
    }
  }

  public void setExtendPosition(Double length) {
    intakeExtendMechanism.setPosition(length);
  }

  public void setShoulderPosition(Double angle) {
    intakeShoulderMechanism.setPosition(angle);
  }

  public void setWristPosition(Double pitch) {
    intakeWristMechanism.setPosition(pitch);
  }

  public double getRequestedWristPosition() {
    return intakeWristMechanism.getRequestedPosition();
  }

  public double getActualWristPosition() {
    return intakeWristMechanism.getActualPosition();
  }

  public Double getRequestedShoulderPosition() {
    return intakeShoulderMechanism.getRequestedPosition();
  }

  public double getActualShoulderElevation() {
    return intakeShoulderMechanism.getActualPosition();
  }

  public Double getRequestedExtendPosition() {
    return intakeExtendMechanism.getRequestedPosition();
  }

  public double getActualExtendPosition() {
    return intakeExtendMechanism.getActualPosition();
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

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_EXTEND, "Extend")
        || shouldMakeAllCANDevices) {
      extend = new CANSparkMaxSendable(MOTORID_INTAKE_EXTEND, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(30).setCoast(true);
      motorSetup.apply(extend);
      addChild("extend1", extend);
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_EXTEND2, "Extend2")
        || shouldMakeAllCANDevices) {
      extend2 = new CANSparkMaxSendable(MOTORID_INTAKE_EXTEND2, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(30).setCoast(true);
      motorSetup.apply(extend2);
      addChild("extend2", extend2);
    }
    
    extendHomeSwitch = new DigitalInput(5);
    addChild("extendHomeSwitch", extendHomeSwitch);

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_WRIST, "Wrist")
        || shouldMakeAllCANDevices) {
      wrist = new CANSparkMaxSendable(MOTORID_INTAKE_WRIST, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(10).setCoast(true);
      motorSetup.apply(wrist);
      addChild("wrist", wrist);
    }

    wristHomeSwitch = new DigitalInput(6);
    addChild("wristHomeSwitch", wristHomeSwitch);
  }

  public void setLocation(IntakeLocation intakeLocation) {
    // logger.info ("Setting intake to {}", intakeLocation);
    intakeShoulderMechanism.setPosition(intakeLocation.getShoulder());
    intakeExtendMechanism.setPosition(intakeLocation.getExtend());
    intakeWristMechanism.setPosition(intakeLocation.getWrist());
  }

  @Override
  public void updateTelemetry() {
    intakeExtendMechanism.updateTelemetry();
    intakeShoulderMechanism.updateTelemetry();
    intakeWristMechanism.updateTelemetry();
  }

}


// :D