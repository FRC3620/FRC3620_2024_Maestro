// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.HasTelemetry;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.Utilities.SlidingWindowStats;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase implements HasTelemetry {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  CANDeviceFinder deviceFinder = RobotContainer.canDeviceFinder;
  TalonFXConfiguration topConfig = new TalonFXConfiguration();
  TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
  private static final String canBusName = "";
  public TalonFX topMotor, bottomMotor;

  public CANSparkMaxSendable elevationMotor;
  RelativeEncoder elevationMotorEncoder, ampBarEncoder;

  public CANSparkMaxSendable ampBarMotor;

  public final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private double requestedWheelSpeed = 0;

  public enum AmpBarPosition {
    HOME, UP
  }
  public static double AmpBarHome=2;
  public static double AmpBarUp=6.25;
  AmpBarPosition currentAmpBarPosition;

  public final static int MOTORID_SHOOTER_BOTTOM = 14;
  public final static int MOTORID_SHOOTER_TOP = 15;
  public final static int MOTORID_SHOOTER_ELEVATION = 16;
  public final static int MOTORID_AMP_BAR=11;

  public SparkPIDController ampBarPID;
  SparkPIDController elevationPid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;
  Timer calibrationTimer;

  // saves the requested position
  double requestedPosition = 0;

  // to save a requested position if encoder is not calibrated
  Double requestedPositionWhileCalibrating = null;

  boolean disabledForDebugging = false;

  final String name = "shooter";

  SlidingWindowStats topSpeedStats, bottomSpeedStats;

  boolean areWeShooting = false;

  double elevationAdjustment = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topConfig.Voltage.PeakForwardVoltage = 12;
    topConfig.Voltage.PeakReverseVoltage = 0;
    bottomConfig.Voltage.PeakForwardVoltage = 12;
    bottomConfig.Voltage.PeakReverseVoltage = 0;
    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    topConfig.Slot0.kP = 1;
    topConfig.Slot0.kI = 0.0;
    topConfig.Slot0.kD = 0.0;
    topConfig.Slot0.kV = 0.12;

    bottomConfig.Slot0.kP = 0.30;
    bottomConfig.Slot0.kI = 0.0;
    bottomConfig.Slot0.kD = 0.0;
    bottomConfig.Slot0.kV = 0.13;

    // Peak output
    topConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
    topConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    topConfig.CurrentLimits.StatorCurrentLimit = 40;
    topConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Peak output
    bottomConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
    bottomConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    bottomConfig.CurrentLimits.StatorCurrentLimit = 50;
    bottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // shooter angle PID parameters and encoder conversion factors
    final double kP = 0.02;
    final double kI = 0.00007;
    final double kD = 0;
    final double kFF = 0;
    final double kIMaxAccum = 0.04;
    final double negOutputLimit = -0.2;
    final double posOutputLimit = 0.2;

    // amp bar pid
    final double aP = 0.07;
    final double aI = 0.0;
    final double aD = 0;
    final double aFF = 0;
    final double aIMaxAccum = 0.0;
    final double aNegOutputLimit = -0.1;
    final double aPosOutputLimit = 0.2;

    // was 9.44 before we swapped out 3:1/5:1 for a 5:1/5:1
    // final double positionConverionFactor = 1.0 * 9.44 * (15.0 / 25.0);
    final double positionConverionFactor = (64 - 18.1) / (64 - 54.93); // difference in angle divided by difference in
                                                                       // motor rotation
    final double velocityConverionFactor = 1.0;

    if (deviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, MOTORID_SHOOTER_BOTTOM, "Bottom Shooter")
        || RobotContainer.shouldMakeAllCANDevices()) {
      bottomMotor = new TalonFX(MOTORID_SHOOTER_BOTTOM, canBusName);
      configMotor("bottom shooter", bottomMotor, bottomConfig);
      addChild("bottom", bottomMotor);
    }

    if (deviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, MOTORID_SHOOTER_TOP, "Top Shooter")
        || RobotContainer.shouldMakeAllCANDevices()) {
      topMotor = new TalonFX(MOTORID_SHOOTER_TOP, canBusName);
      configMotor("top shooter", topMotor, topConfig);
      addChild("top", topMotor);
    }

    if (deviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_SHOOTER_ELEVATION, "Shooter Elaevation")
        || RobotContainer.shouldMakeAllCANDevices()) {
      elevationMotor = new CANSparkMaxSendable(MOTORID_SHOOTER_ELEVATION, MotorType.kBrushless);
      MotorSetup setup = new MotorSetup().setCurrentLimit(10).setCoast(false);
      setup.apply(elevationMotor);
      addChild("elevationMotor", elevationMotor);

      MotorSetup motorSetup = new MotorSetup().setCoast(false).setCurrentLimit(80);
      motorSetup.apply(elevationMotor);
      elevationMotorEncoder = elevationMotor.getEncoder();
      elevationMotorEncoder.setPositionConversionFactor(positionConverionFactor);
      elevationMotorEncoder.setVelocityConversionFactor(velocityConverionFactor);

      elevationPid = elevationMotor.getPIDController();
      elevationPid.setP(kP);
      elevationPid.setI(kI);
      elevationPid.setD(kD);
      elevationPid.setFF(kFF);
      var err = elevationPid.setIMaxAccum(kIMaxAccum, 0);
      if (err != REVLibError.kOk) {
        logger.error("Could not set elevation kImaxAccum: {}", err);
      }
      elevationPid.setOutputRange(negOutputLimit, posOutputLimit);
    }
    
    
    if (deviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, MOTORID_AMP_BAR, "Shooter Amp Bar")
        || RobotContainer.shouldMakeAllCANDevices()) {
      ampBarMotor = new CANSparkMaxSendable(MOTORID_AMP_BAR, MotorType.kBrushless);//brushless??
      MotorSetup setup = new MotorSetup().setCurrentLimit(10).setCoast(false).setInverted(true);
      setup.apply(ampBarMotor);
      addChild("AmpBar", ampBarMotor);

      ampBarEncoder = ampBarMotor.getEncoder();
      ampBarEncoder.setPositionConversionFactor(1.0);
      ampBarEncoder.setVelocityConversionFactor(1.0);
      ampBarEncoder.setPosition(0.0);

      ampBarPID = ampBarMotor.getPIDController();
      ampBarPID.setP(aP);
      ampBarPID.setI(aI);
      ampBarPID.setD(aD);
      ampBarPID.setFF(aFF);
      var err = ampBarPID.setIMaxAccum(aIMaxAccum, 0);
      if (err != REVLibError.kOk) {
        logger.error("Could not set amp bar kImaxAccum: {}", err);
      }
      ampBarPID.setOutputRange(aNegOutputLimit, aPosOutputLimit);
    }
    
    topSpeedStats = new SlidingWindowStats(100);
    bottomSpeedStats = new SlidingWindowStats(100);

    updateDashboardElevationAdjustment();
  }

  void configMotor(String motorName, TalonFX motor, TalonFXConfiguration configuration) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(configuration);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs to shooter." + motorName + ", error code: " + status.toString());
    }
    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setRequestedWheelSpeed(double speed) {
    SmartDashboard.putNumber(name + ".wheels.requested_velocity", speed);
    if (speed > 4500) {
      areWeShooting = true;
    } else {
      areWeShooting = false;
    }
    this.requestedWheelSpeed = speed / 60; // convert RPM to RPS

  }

  public double getTopMotorVelocity() {
    return getMotorVelocity(topMotor);
  }

  public double getBottomMotorVelocity() {
    return getMotorVelocity(bottomMotor);
  }

  double getMotorVelocity(TalonFX motor) {
    if (motor != null) {
      return motor.getVelocity().getValueAsDouble();
    } else {
      return 0;
    }
  }

  Double manuallySetPower = null;

  public void setWheelPower(Double p) {
    manuallySetPower = p;
  }

  public void setAmpBarPosition(AmpBarPosition ampBarPosition) {
    currentAmpBarPosition= ampBarPosition;
    SmartDashboard.putString(name + ".ampbar.request_position_t", currentAmpBarPosition.toString());
    double position;
    if (ampBarPosition == AmpBarPosition.HOME) {
      position = AmpBarHome;
    }else{//Up
      position= AmpBarUp;
    }
    SmartDashboard.putNumber (name + ".ampbar.requested_position", position);
    if (ampBarPID != null) {
      ampBarPID.setReference(position, ControlType.kPosition);
    }
  }

  public void bumpAmpBar(double bump){
    if(currentAmpBarPosition== AmpBarPosition.UP){
      AmpBarUp+=bump;
      setAmpBarPosition(AmpBarPosition.UP);
      logger.info("amp bar bumped to {}", AmpBarUp);
    }
  }

  public Double getAmpBarPosition() {
    if (ampBarEncoder != null) {
      return ampBarEncoder.getPosition();
    } else {
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("areWeShooting", areWeShooting);

    if (topMotor != null) {
      if (manuallySetPower != null) {
        topMotor.set(manuallySetPower);
      } else {
        if (requestedWheelSpeed == 0) { // keep PID from kicking in when we want to stop
          topMotor.stopMotor();
        } else {
          topMotor.setControl(m_voltageVelocity.withVelocity(requestedWheelSpeed)); // RPS
        }
      }
    }

    if (bottomMotor != null) {
      if (manuallySetPower != null) {
        bottomMotor.set(manuallySetPower);
      } else {
        if (requestedWheelSpeed == 0) {
          bottomMotor.stopMotor();
        } else {
          bottomMotor.setControl(m_voltageVelocity.withVelocity(requestedWheelSpeed * 0.9));
        }
      }
    }

    double newElevationAdjustment = readDashboardElevationAdjustment();
    if (newElevationAdjustment != elevationAdjustment) {
      elevationAdjustment = newElevationAdjustment;
      if (encoderCalibrated && ! disabledForDebugging) {
        elevationPid.setReference(requestedPosition + elevationAdjustment, ControlType.kPosition);
      }
    }

    // only do something if we actually have a motor
    if (elevationMotor != null) {
      if (elevationMotorEncoder != null) { // if there is an encoder, display these
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) {
            // If the robot is running, and the encoder is "not calibrated," run motor very
            // slowly towards the switch
            setElevationPower(0.08); 
            if (calibrationTimer == null) {
              // we need to calibrate and we have no timer. make one and start it
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              // we have a timer, has the motor had power long enough to spin up
              if (calibrationTimer.get() > 0.5) {
                if (Math.abs(elevationMotorEncoder.getVelocity()) < 1) {
                  // motor is not moving, hopefully it's against the stop
                  encoderCalibrated = true;
                  setElevationPower(0.0);
                  elevationMotorEncoder.setPosition(64);
                  setElevationPosition(64);
                  setAmpBarPosition(AmpBarPosition.HOME);

                  // If there was a requested position while we were calibrating, go there
                  if (requestedPositionWhileCalibrating != null) {
                    setElevationPosition(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }

                }
              }
            }
          }
        }
      }
    }
  }

  public void setElevationPosition(double position) {
    position = MathUtil.clamp(position, 17, 63); // high end is a little short of the stop
    SmartDashboard.putNumber(name + ".elevation.requestedPosition", position);
    requestedPosition = position;
    if (encoderCalibrated) {
      if (!disabledForDebugging) {
        elevationPid.setReference(position + elevationAdjustment, ControlType.kPosition);
      }
    } else {
      requestedPositionWhileCalibrating = position;
    }
  }

  public double getRequestedShooterElevation() {
    return requestedPosition;
  }

  public double getActualElevationPosition() {
    if (elevationMotorEncoder != null) return elevationMotorEncoder.getPosition();
    return 0;
  }

  void setElevationPower(double power) {
    if (!disabledForDebugging) {
      elevationMotor.set(power);
    }
  }

  public void setSpeedAndAngle(ShooterSpeedAndAngle speedAndAngle) {
    // SPAM SPAM SPAM SPAM WONDERFUL SPAM
    // logger.info("Setting intake to {}", speedAndAngle);
    setRequestedWheelSpeed(speedAndAngle.speed);
    setElevationPosition(speedAndAngle.position);
    requestedPosition = speedAndAngle.position;
  }

  @Override
  public void updateTelemetry() {
    putMotorInformationToDashboard("top", topMotor, topSpeedStats);
    putMotorInformationToDashboard("bottom", bottomMotor, bottomSpeedStats);
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);
    if (elevationMotor != null) {
      SmartDashboard.putNumber(name + ".elevation.current", elevationMotor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".elevation.power", elevationMotor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".elevation.temperature", elevationMotor.getMotorTemperature());
      SmartDashboard.putNumber(name + ".elevation.IAccum", elevationPid.getIAccum());

      if (elevationMotorEncoder != null) { // if there is an encoder, display these
        double velocity = elevationMotorEncoder.getVelocity();
        double position = elevationMotorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".elevation.velocity", velocity);
        SmartDashboard.putNumber(name + ".elevation.position", position);
      }
    }
    if (ampBarMotor != null) {
      SmartDashboard.putNumber(name + ".ampbar.current", ampBarMotor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".ampbar.power", ampBarMotor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".ampbar.temperature", ampBarMotor.getMotorTemperature());

      if (ampBarEncoder != null) { // if there is an encoder, display these
        double velocity = ampBarEncoder.getVelocity();
        double position = ampBarEncoder.getPosition();
        SmartDashboard.putNumber(name + ".ampbar.velocity", velocity);
        SmartDashboard.putNumber(name + ".ampbar.position", position);
      }
    }
  }

  void putMotorInformationToDashboard(String motorName, TalonFX motor, SlidingWindowStats slidingWindowStats) {
    if (motor != null) {
      String prefix = name + "." + motorName;
      double currentVelocity = motor.getVelocity().getValueAsDouble() * 60; // convert RPS to RPM
      SmartDashboard.putNumber(prefix + ".velocity", currentVelocity);
      SmartDashboard.putNumber(prefix + ".closedLoopOutput", motor.getClosedLoopOutput().getValueAsDouble());
      SmartDashboard.putNumber(prefix + ".torqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber(prefix + ".appliedPower", motor.get());
      SmartDashboard.putNumber(prefix + ".supplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber(prefix + ".statorCurrent", motor.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber(prefix + ".temperature", motor.getDeviceTemp().getValueAsDouble());

      slidingWindowStats.addValue(currentVelocity);
      SmartDashboard.putNumber(name + "." + motorName + ".sliding.mean", slidingWindowStats.getMean());
      SmartDashboard.putNumber(name + "." + motorName + ".sliding.stdev", slidingWindowStats.getStdDev());
      SmartDashboard.putNumber(name + "." + motorName + ".sliding.flyers", slidingWindowStats.getFlyers());
    }
  }

  void updateDashboardElevationAdjustment() {
    SmartDashboard.putNumber(name + ".elevationAdjustment", elevationAdjustment);
  }

  double readDashboardElevationAdjustment() {
    return SmartDashboard.getNumber(name + ".elevationAdjustment", elevationAdjustment);
  }

  public double getElevationAdjustment() {
    return elevationAdjustment;
  }

}
