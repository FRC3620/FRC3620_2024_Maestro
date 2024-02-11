// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.CANSparkMax;
import org.usfirst.frc3620.misc.CANSparkMaxSendableWrapper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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

  //public CANSparkMaxSendable roll;

  public CANSparkMaxSendable wrist;
  public DigitalInput wristHomeSwitch;

  public CANSparkMaxSendable rollers;
  public DigitalInput gamePieceObtained;

  // public RelativeEncoder rollersEncoder;

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
  }

  public void setExtendPosition(double length) {
    intakeExtendMechanism.setPosition(length);
  }

  public void setExtendPower(double power) {
    intakeExtendMechanism.setPower(power);
  }

  // public void disableExtension() {
  // intakeExtendMechanism.disable();
  // }

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

  /*
   * public double getAdjustedRequestedExtension() {
   * return intakeExtendMechanism.getAdjustedRequestedExtension();
   * }
   */

  public void recalibrataePitch(boolean forward) {
    recalibrataePitch(forward);
  }

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
    boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 13, "Intake Elevation") || shouldMakeAllCANDevices) {
      shoulder = new CANSparkMaxSendable(13, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(20).setCoast(false);
      motorSetup.apply(shoulder);
      addChild("elevation", shoulder);
    }

    shoulderEncoder = new DutyCycleEncoder(7);
    addChild("shoulderEncoder", shoulderEncoder);

    /*
     * if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 14, "Elevate2")
     * || shouldMakeAllCANDevices) {
     * elevation2 = new CANSparkMaxSendable(10, MotorType.kBrushless);
     * MotorSetup.resetMaxToKnownState(elevation2, true);
     * elevation2.setSmartCurrentLimit(40);
     * elevation2.setIdleMode(IdleMode.kBrake);
     * addChild("elevate2", elevation2);
     * }
     */

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 10, "Extend") || shouldMakeAllCANDevices) {
      extend = new CANSparkMaxSendable(10, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(false).setCurrentLimit(10).setCoast(true);
      motorSetup.apply(extend);
      addChild("extend", extend);
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 11, "Extend2") || shouldMakeAllCANDevices) {
      extend2 = new CANSparkMaxSendable(11, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(10).setCoast(true);
      motorSetup.apply(extend2);
      addChild("extend2", extend2);
    }

    extendHomeSwitch = new DigitalInput(5);
    addChild("extendHomeSwitch", extendHomeSwitch);

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 12, "Wrist") || shouldMakeAllCANDevices) {
      wrist = new CANSparkMaxSendable(12, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(10).setCoast(false);
      motorSetup.apply(wrist);
      addChild("wrist", wrist);
    }

    wristHomeSwitch = new DigitalInput(6);
    addChild("wristHomeSwitch", wristHomeSwitch);

     /* wristMotorEncoder = wrist.getEncoder();
      // 360 to convert from rotations to degrees
      // 10 is a 5:1 gearbox and a 2:1 pulley reduction
      wristMotorEncoder.setPositionConversionFactor(360.0 / 10.0);
      logger.info("Wrist motor position scale = {}", wristMotorEncoder.getPositionConversionFactor());
    
    */

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 9, "Rollers") || shouldMakeAllCANDevices) {
      rollers = new CANSparkMaxSendable(9, MotorType.kBrushless);
      MotorSetup motorSetup = new MotorSetup().setInverted(true).setCurrentLimit(20).setCoast(false);
      motorSetup.apply(rollers);
      addChild("roller", rollers);
    }

    gamePieceObtained = new DigitalInput(9);




    //shoulderEncoder = new AnalogInput(4);
    //addChild("shoulderEncoder", shoulderEncoder);

   // wristEncoder = new Encoder(5, 6);
    //addChild("wristEncoder", wristEncoder);
  }

  public void setLocation(IntakeLocation intakeLocation) {
    intakeShoulderMechanism.setPosition(intakeLocation.getShoulder());
    intakeExtendMechanism.setPosition(intakeLocation.getExtend());
    intakeWristMechanism.setPosition(intakeLocation.getWrist());
  }
}

// :D
