// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  TalonFXConfiguration configs=new TalonFXConfiguration();
  private static final String canBusName = "";
  private final TalonFX m_fx = new TalonFX(20, canBusName);
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final TalonFX m_fllr = new TalonFX(21, canBusName);
  private double speed= 0.6;
  
   
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
   configs.Voltage.PeakForwardVoltage=8;
   configs.Voltage.PeakReverseVoltage=-8;
       /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
       configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
       configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
       configs.Slot0.kD = 0.0001;// A change of 1 rotation per second squared results in 0.01 volts output
       configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 10;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -10;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
 
  }

   public void setSpeed(double speed){
    this.speed=speed;
   }
  @Override
  public void periodic() {
    double currentVelocity= m_fllr.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("shooter Velocity:", currentVelocity);
    double getClosedLoopOutput=m_fllr.getClosedLoopOutput().getValueAsDouble();
    double getTorqueCurrent=m_fllr.getTorqueCurrent().getValueAsDouble();
    SmartDashboard.putNumber("TC", getTorqueCurrent);
    SmartDashboard.putNumber("CLO", getClosedLoopOutput);
    m_fx.setControl(m_voltageVelocity.withVelocity(speed));
    m_fllr.setControl(m_voltageVelocity.withVelocity(speed));
  }
}
