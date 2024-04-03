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
         rAngle=87.068+-11.9696*distanceFt+ .732323  *Math.pow(distanceFt, 2);
        }else if(distanceFt<10.5){
            rAngle= 51.8587+-2.098*distanceFt+0.0404135*Math.pow(distanceFt, 2);
        }else{
            rAngle=98.58167+-9.71519*distanceFt+0.347431*Math.pow(distanceFt, 2);
        }
    ShooterSpeedAndAngle shooterSpeedAndAngle= new ShooterSpeedAndAngle(0, rAngle);

        return shooterSpeedAndAngle;
    }
    public static ShooterSpeedAndAngle CalculaiteAngleM(Double distanceM){
        distanceM*=3.28;
        return CalculaiteAngleFt(distanceM);
    }
}
