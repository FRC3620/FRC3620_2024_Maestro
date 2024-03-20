// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ShooterSpeedAndAngle;

/** Add your docs here. */
public class ShooterCalcutlaiter {

    public static ShooterSpeedAndAngle CalculaiteAngleFt(Double distanceFt){
        double rAngle=0;
        if(distanceFt<6.8){
         rAngle=82.512+-10.166*distanceFt+ .55402  *Math.pow(distanceFt, 2);
        }else if(distanceFt<10.5){
            rAngle= 58.773+-3.741*distanceFt+0.12249*Math.pow(distanceFt, 2);
        }else{
            rAngle=74.051+-6.106*distanceFt+0.20918*Math.pow(distanceFt, 2);
        }
    ShooterSpeedAndAngle shooterSpeedAndAngle= new ShooterSpeedAndAngle(0, rAngle);

        return shooterSpeedAndAngle;
    }
    public static ShooterSpeedAndAngle CalculaiteAngleM(Double distanceM){
        distanceM*=3.28;
        return CalculaiteAngleFt(distanceM);
    }
}
