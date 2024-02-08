// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  CANDeviceFinder deviceFinder = RobotContainer.canDeviceFinder;
  TalonFXConfiguration configs = new TalonFXConfiguration();
  private static final String canBusName = "";
  public TalonFX topMotor, bottomMotor;
  public final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private double speed = 0.6;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0;// A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.0; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second

    // Peak output of 10 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10;
    if (deviceFinder.isDevicePresent(CANDeviceType.TALON, 15, "Bottom Shooter") || RobotContainer.shouldMakeAllCANDevices()) {
      bottomMotor = new TalonFX(15, canBusName);
      configMotor("bottom shooter", bottomMotor);
    }

    if (deviceFinder.isDevicePresent(CANDeviceType.TALON, 14, "Top Shooter") || RobotContainer.shouldMakeAllCANDevices()) {
      topMotor = new TalonFX(14, canBusName);
      configMotor("top shooter", topMotor);
    }

  }

  void configMotor(String motorName, TalonFX motor) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs to shooter." + motorName + ", error code: " + status.toString());
    }
    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public double getMotorVelocity(TalonFX motor) {
    if (motor != null) {
      return motor.getVelocity().getValueAsDouble();
    } else {
      return 0;
    }
  }

  void putMotorInformationToDashboard(String motorName, TalonFX motor) {
    if (motor != null) {
      double currentVelocity = motor.getVelocity().getValueAsDouble();
      SmartDashboard.putNumber("shooter." + motorName + ".velocity", currentVelocity);
      double getClosedLoopOutput = motor.getClosedLoopOutput().getValueAsDouble();
      SmartDashboard.putNumber("shooter." + motorName + ".closedLoopOutput", getClosedLoopOutput);
      double getTorqueCurrent = motor.getTorqueCurrent().getValueAsDouble();
      SmartDashboard.putNumber("shooter." + motorName + ".torqueCurrent", getTorqueCurrent);
    }
  }

  @Override
  public void periodic() {
    if (topMotor != null) {
      putMotorInformationToDashboard("top", topMotor);
      topMotor.setControl(m_voltageVelocity.withVelocity(speed));
    }

    if (bottomMotor != null) {
      putMotorInformationToDashboard("bottom", bottomMotor);
      bottomMotor.setControl(m_voltageVelocity.withVelocity(speed));
    }
  }
}
