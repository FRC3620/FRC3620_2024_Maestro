// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.Utilities;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElevationSubsystem extends SubsystemBase {
  /** Creates a new ArmElevationSubsystem. */
  
    CANSparkMaxSendable motor;
    RelativeEncoder motorEncoder;


    int elevateEncoderValueAt90Degrees;

    final double kP = 0; //change PID later
    final double kI = 0;
    final double kD = 0;

    final PIDController m_pidController = new PIDController(kP, kI, kD);

    double elevationOffset;

    double requestedPosition = 0; //degrees change later

    final String name = "Arm Elevate";
  
    public ArmElevationSubsystem(CANSparkMaxSendable motor){

      this.motor = motor;
      this.motorEncoder = motor.getEncoder();
      

       }
      
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevation(double elevation) {
    elevation = MathUtil.clamp(elevation, -45, 200);
    SmartDashboard.putNumber(name + ".requestedElevation", elevation);
    requestedPosition = elevation;

    m_pidController.setSetpoint(elevation);
  }

  public double getRequestedElevation() {
    return requestedPosition;
  }

  public void setPower(double power) {
    motor.set(power);
  }

  public double getCurrentPosition(){
    if (motorEncoder == null) return 0;
		double motorEncoderValue = motorEncoder.getPosition();
    // converting heading from tics (ranging from 0 to 4095) to degrees
		double position = (motorEncoderValue - elevateEncoderValueAt90Degrees)*(360.0/4096.0) + 90;
    // get it into the -180..180 range
    position = Utilities.normalizeAngle(position);
    // get it into the -90..270 range
    if (position< -90){
      position = position + 360;
    }
		return position;
	}
}


