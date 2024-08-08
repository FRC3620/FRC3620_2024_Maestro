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
         rAngle=129.2961186+-28.91633242*distanceFt+ 2.272542078  *Math.pow(distanceFt, 2);
        }else if(distanceFt<10.5){
            rAngle= 48.73426573+-1.607778187*distanceFt+-0.01226843332*Math.pow(distanceFt, 2);
        }else{
            rAngle=50.56925208+-2.638504155*distanceFt+0.06925207756*Math.pow(distanceFt, 2);
        }
    ShooterSpeedAndAngle shooterSpeedAndAngle= new ShooterSpeedAndAngle(0, rAngle);

        return shooterSpeedAndAngle;
    }
    public static ShooterSpeedAndAngle CalculaiteAngleM(Double distanceM){
        distanceM*=3.28;
        return CalculaiteAngleFt(distanceM);
    }
}
