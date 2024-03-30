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

public class ShooterWheelsAndAmpBarSubsystem extends SubsystemBase implements HasTelemetry {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  CANDeviceFinder deviceFinder = RobotContainer.canDeviceFinder;
  TalonFXConfiguration topConfig = new TalonFXConfiguration();
  TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
  private static final String canBusName = "";
  public TalonFX topMotor, bottomMotor;

  public CANSparkMaxSendable ampBarMotor;
  RelativeEncoder ampBarEncoder;

  public final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private double requestedWheelSpeed = 0;

  public enum AmpBarPosition {
    HOME, UP
  }
  public static double AmpBarHome = 0;
  public static double AmpBarUp = 5;
  AmpBarPosition currentAmpBarPosition;

  public final static int MOTORID_SHOOTER_BOTTOM = 14;
  public final static int MOTORID_SHOOTER_TOP = 15;
  public final static int MOTORID_AMP_BAR = 11;

  public SparkPIDController ampBarPID;

  boolean disabledForDebugging = false;

  final String name = "shooter";

  SlidingWindowStats topSpeedStats, bottomSpeedStats;

  boolean areWeShooting = false;

  double elevationAdjustment = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterWheelsAndAmpBarSubsystem() {
    topConfig.Voltage.PeakForwardVoltage = 12;
    topConfig.Voltage.PeakReverseVoltage = 12;
    bottomConfig.Voltage.PeakForwardVoltage = 12;
    bottomConfig.Voltage.PeakReverseVoltage = 12;
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
    topConfig.TorqueCurrent.PeakReverseTorqueCurrent = 20;
    topConfig.CurrentLimits.StatorCurrentLimit = 40;
    topConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Peak output
    bottomConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
    bottomConfig.TorqueCurrent.PeakReverseTorqueCurrent = 20;
    bottomConfig.CurrentLimits.StatorCurrentLimit = 50;
    bottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // amp bar pid
    final double aP = 0.07;
    final double aI = 0.0;
    final double aD = 0;
    final double aFF = 0;
    final double aIMaxAccum = 0.0;
    final double aNegOutputLimit = -0.3;
    final double aPosOutputLimit = 0.2;

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

    setAmpBarPosition(AmpBarPosition.HOME);
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
  }

  public void setRequestedWheelSpeed(ShooterSpeedAndAngle speedAndAngle) {
    setRequestedWheelSpeed(speedAndAngle.speed);
  }

  @Override
  public void updateTelemetry() {
    putMotorInformationToDashboard("top", topMotor, topSpeedStats);
    putMotorInformationToDashboard("bottom", bottomMotor, bottomSpeedStats);

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
