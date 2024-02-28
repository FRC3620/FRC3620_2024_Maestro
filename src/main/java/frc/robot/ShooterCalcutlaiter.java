// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ShooterSpeedAndAngle;

/** Add your docs here. */
public class ShooterCalcutlaiter {

    public static ShooterSpeedAndAngle CalculaiteAngle(Double distance){
        double rAngle=0;
        if(distance<7){
         rAngle=109.125+-19.5*distance+ 1.375  *Math.pow(distance, 2);
        }else if(distance<11){
            rAngle= 50.5+-1.5*distance+0*Math.pow(distance, 2);
        }else{
            rAngle=42.25+-0.75*distance+0*Math.pow(distance, 2);
        }
    ShooterSpeedAndAngle shooterSpeedAndAngle= new ShooterSpeedAndAngle(0, rAngle);

        return shooterSpeedAndAngle;
    }
}
