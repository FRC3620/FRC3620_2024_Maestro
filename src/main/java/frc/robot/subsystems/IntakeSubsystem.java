package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public IntakeExtendMechanism intakeExtendMechanism;
  public IntakeShoulderMechanism intakeShoulderMechanism;
  public IntakeWristMechanism intakeWristMechanism;
  public IntakeRollersMechanism intakeRollerMechanism;

  public CANSparkMaxSendable shoulder;
  public DutyCycleEncoder shoulderEncoder;

  public CANSparkMaxSendable extend;
  public DigitalInput extendHomeSwitch;

  public CANSparkMaxSendable extend2;

  public CANSparkMaxSendable wrist;
  public DigitalInput wristHomeSwitch;

  public CANSparkMaxSendable rollers;
  public DigitalInput gamePieceObtained;

  double temporaryPosition;

  /** Creates a new ArmSubsystem. */
  public IntakeSubsystem() {
    setupMotors();
    intakeExtendMechanism = new IntakeExtendMechanism(extend, extend2);
    intakeShoulderMechanism = new IntakeShoulderMechanism(shoulder, shoulderEncoder);
    intakeWristMechanism = new IntakeWristMechanism(wrist, wristHomeSwitch);
    intakeRollerMechanism = new IntakeRollersMechanism(rollers, gamePieceObtained);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeExtendMechanism.periodic();
    intakeShoulderMechanism.periodic();
    intakeWristMechanism.periodic();
    intakeRollerMechanism.periodic();

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
  }

  boolean cantElevate() {
    double extendPosition;
    extendPosition = intakeExtendMechanism.getActualPosition();
    // change 1 to actual restraint
    if (extendPosition > 1) {
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

  public void setExtendPosition(double length) {
    intakeExtendMechanism.setPosition(length);
  }

  public void setExtendPower(double power) {
    intakeExtendMechanism.setPower(power);
  }

  public void setShoulderPosition(double angle) {
    intakeShoulderMechanism.setPosition(angle);
  }

  public void setIntakePower(double power) {
    intakeShoulderMechanism.setPower(power);
  }

  public void setWristPosition(double pitch) {
    intakeWristMechanism.setPosition(pitch);
  }

  public void setWristPower(double power) {
    intakeWristMechanism.setPower(power);
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

  public double getRequestedWristPosition() {
    return intakeWristMechanism.getRequestedPosition();
  }

  public double getActualWristPosition() {
    return intakeWristMechanism.getActualPosition();
  }

  public double getRequestedShoulderPosition() {
    return intakeShoulderMechanism.getRequestedPosition();
  }

  public double getActualShoulderElevation() {
    return intakeShoulderMechanism.getActualPosition();
  }

  public double getRequestedExtendPosition() {
    return intakeExtendMechanism.getRequestedPosition();
  }

  public double getActualExtendPosition() {
    return intakeExtendMechanism.getActualPosition();
  }

  public void recalibrataePitch(boolean forward) {
    recalibrataePitch(forward);
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
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(20).setCoast(false);
      motorSetup.apply(shoulder);
      addChild("elevation", shoulder);
    }

    shoulderEncoder = new DutyCycleEncoder(7);
    addChild("shoulderEncoder", shoulderEncoder);

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_EXTEND, "Extend")
        || shouldMakeAllCANDevices) {
      extend = new CANSparkMaxSendable(MOTORID_INTAKE_EXTEND, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(10).setCoast(true);
      motorSetup.apply(extend);
      addChild("extend1", extend);
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_EXTEND2, "Extend2")
        || shouldMakeAllCANDevices) {
      extend2 = new CANSparkMaxSendable(MOTORID_INTAKE_EXTEND2, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(10).setCoast(true);
      motorSetup.apply(extend2);
      addChild("extend2", extend2);
    }

    extendHomeSwitch = new DigitalInput(5);
    addChild("extendHomeSwitch", extendHomeSwitch);

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_WRIST, "Wrist")
        || shouldMakeAllCANDevices) {
      wrist = new CANSparkMaxSendable(MOTORID_INTAKE_WRIST, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(10).setCoast(false);
      motorSetup.apply(wrist);
      addChild("wrist", wrist);
    }

    wristHomeSwitch = new DigitalInput(6);
    addChild("wristHomeSwitch", wristHomeSwitch);

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_INTAKE_ROLLERS, "Rollers")
        || shouldMakeAllCANDevices) {
      rollers = new CANSparkMaxSendable(MOTORID_INTAKE_ROLLERS, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(20).setCoast(false);
      motorSetup.apply(rollers);
      addChild("roller", rollers);
    }

    gamePieceObtained = new DigitalInput(9);
  }

  public void setLocation(IntakeLocation intakeLocation) {
    intakeShoulderMechanism.setPosition(intakeLocation.getShoulder());
    intakeExtendMechanism.setPosition(intakeLocation.getExtend());
    intakeWristMechanism.setPosition(intakeLocation.getWrist());
  }
}

// :D
