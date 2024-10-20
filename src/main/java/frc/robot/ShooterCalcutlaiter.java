// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ShooterSpeedAndAngle;

/** Add your docs here. */
public class ShooterCalcutlaiter {

    public static ShooterSpeedAndAngle CalculaiteAngleFt(Double distanceFt){
        double rAngle=0;
        if(distanceFt<7.2){
         rAngle=116.552+-22.811*distanceFt+ 1.65354  *Math.pow(distanceFt, 2);
        }else if(distanceFt<11.15){
            rAngle= 54.416+-2.711*distanceFt+0.06038*Math.pow(distanceFt, 2);
        }else{
            rAngle=65.169+-4.510*distanceFt+0.13524*Math.pow(distanceFt, 2);
        }
    ShooterSpeedAndAngle shooterSpeedAndAngle= new ShooterSpeedAndAngle(0, rAngle);

        return shooterSpeedAndAngle;
    }
    public static ShooterSpeedAndAngle CalculaiteAngleM(Double distanceM){
        distanceM*=3.28;
        return CalculaiteAngleFt(distanceM);
    }
}
