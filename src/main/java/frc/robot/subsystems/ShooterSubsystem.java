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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
  CANSparkMaxSendable elevationMotor;
  RelativeEncoder elevationMotorEncoder;

  public final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private double speed = 0.6;

  public final static int MOTORID_SHOOTER_BOTTOM = 14;
  public final static int MOTORID_SHOOTER_TOP = 15;
  public final static int MOTORID_SHOOTER_ELEVATION = 16;

  SparkPIDController pid = null;

  // Robot is set to "not calibrated" by default
  boolean encoderCalibrated = false;
  Timer calibrationTimer;

  // saves the requested position
  double requestedPosition = 0;

  // to save a requested position if encoder is not calibrated
  Double requestedPositionWhileCalibrating = null;

  boolean disabledForDebugging = false;

  final String name = "shooter";

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topConfig.Voltage.PeakForwardVoltage = 12;
    topConfig.Voltage.PeakReverseVoltage = -12;
    bottomConfig.Voltage.PeakForwardVoltage=12;
    bottomConfig.Voltage.PeakReverseVoltage=-12;
    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    topConfig.Slot0.kP = 0.22; // An error of 1 rotation per second results in 2V output
    topConfig.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    topConfig.Slot0.kD = 0.0;// A change of 1 rotation per second squared results in 0.01 volts output
    topConfig.Slot0.kV = 0.135; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                            // volts / Rotation per second
    
    bottomConfig.Slot0.kP = 0.22; // An error of 1 rotation per second results in 2V output
    bottomConfig.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    bottomConfig.Slot0.kD = 0.0;// A change of 1 rotation per second squared results in 0.01 volts output
    bottomConfig.Slot0.kV = 0.15; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                            // volts / Rotation per second
    // Peak output of 10 amps
    topConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
    topConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Peak output of 10 amps
    bottomConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
    bottomConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // PID parameters and encoder conversion factors
    final double kP = 0.0125; //
    final double kI = 0;
    final double kD = 0;
    final double kFF = 0; // define FF
    final double outputLimit = 0.2; // the limit that the power cannot exceed

    final double positionConverionFactor = 1.0 * 9.44;
    final double velocityConverionFactor = 1.0;

    if (deviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, MOTORID_SHOOTER_BOTTOM, "Bottom Shooter")
        || RobotContainer.shouldMakeAllCANDevices()) {
      bottomMotor = new TalonFX(MOTORID_SHOOTER_BOTTOM, canBusName);
      configMotor("bottom shooter", bottomMotor,bottomConfig);
      addChild("bottom", bottomMotor);
    }

    if (deviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, MOTORID_SHOOTER_TOP, "Top Shooter")
        || RobotContainer.shouldMakeAllCANDevices()) {
      topMotor = new TalonFX(MOTORID_SHOOTER_TOP, canBusName);
      configMotor("top shooter", topMotor,topConfig);
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

      pid = elevationMotor.getPIDController();
      pid.setP(kP); //
      pid.setI(kI); //
      pid.setD(kD); //
      pid.setFF(kFF); //
      pid.setOutputRange(-outputLimit, outputLimit);
    }
  }

  void configMotor(String motorName, TalonFX motor,TalonFXConfiguration configuration) {
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

  public void setSpeed(double speed) {
    SmartDashboard.putNumber (name + ".wheels.requested_velocity", speed);
    this.speed = speed / 60; // convert RPM to RPS
  }

  public double getMotorVelocity(TalonFX motor) {
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

  @Override
  public void periodic() {
    if (topMotor != null) {
      if (manuallySetPower != null) {
        topMotor.set(manuallySetPower);
      } else {
        topMotor.setControl(m_voltageVelocity.withVelocity(speed));  // RPS
      }
    }

    if (bottomMotor != null) {
      if (manuallySetPower != null) {
        bottomMotor.set(manuallySetPower);
      } else {
        bottomMotor.setControl(m_voltageVelocity.withVelocity(speed));
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
                  elevationMotorEncoder.setPosition(63.7);
                  setElevationPosition(63.7);

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
    position = MathUtil.clamp(position, 29,  62.7);  // high end is a little short of the stop
    SmartDashboard.putNumber(name + ".elevation.requestedPosition", position);
    requestedPosition = position;
    if (encoderCalibrated) {
      if (!disabledForDebugging) {
        pid.setReference(position, ControlType.kPosition);
      }
    } else {
      requestedPositionWhileCalibrating = position;
    }
  }

  void setElevationPower(double power) {
    if (!disabledForDebugging) {
      elevationMotor.set(power);
    }
  }

  public void setSpeedAndAngle(ShooterSpeedAndAngle speedAndAngle) {
    // SPAM SPAM SPAM SPAM WONDERFUL SPAM
    // logger.info("Setting intake to {}", speedAndAngle);
    setSpeed(speedAndAngle.speed);
    setElevationPosition(speedAndAngle.position);
    requestedPosition=speedAndAngle.position;
  }

  @Override
  public void updateTelemetry() {
    putMotorInformationToDashboard("top", topMotor);
    putMotorInformationToDashboard("bottom", bottomMotor);
    SmartDashboard.putBoolean(name + ".calibrated", encoderCalibrated);
    if (elevationMotor != null) {
      SmartDashboard.putNumber(name + ".elevation.current", elevationMotor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".elevation.power", elevationMotor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".elevation.temperature", elevationMotor.getMotorTemperature());

      if (elevationMotorEncoder != null) { // if there is an encoder, display these
        double velocity = elevationMotorEncoder.getVelocity();
        double position = elevationMotorEncoder.getPosition();
        SmartDashboard.putNumber(name + ".elevation.velocity", velocity);
        SmartDashboard.putNumber(name + ".elevation.position", position);
      }
    }
  }

  void putMotorInformationToDashboard(String motorName, TalonFX motor) {
    if (motor != null) {
      double currentVelocity = motor.getVelocity().getValueAsDouble() * 60; // convert RPS to RPM
      SmartDashboard.putNumber(name + "." + motorName + ".velocity", currentVelocity);
      double getClosedLoopOutput = motor.getClosedLoopOutput().getValueAsDouble();
      SmartDashboard.putNumber(name + "." + motorName + ".closedLoopOutput", getClosedLoopOutput);
      double getTorqueCurrent = motor.getTorqueCurrent().getValueAsDouble();
      SmartDashboard.putNumber(name + "." + motorName + ".torqueCurrent", getTorqueCurrent);
      SmartDashboard.putNumber(name + "." + motorName + ".appliedPower", motor.get());
    }
  }

}
