// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


/** Add your docs here. */
public class IntakeLocation {
    //public static IntakeLocation coneHighLocation = new IntakeLocation(44, 35.5, -75);
    double shoulder;
    double extend;
    double wrist;

    public IntakeLocation(double shoulder, double extend, double wrist) {
        this.shoulder = shoulder;
        this.extend = extend;
        this.wrist = wrist;
    }

    public double getShoulder() {
        return shoulder;
    }

    public double getWrist() {
        return wrist;
    }

    public double getExtend() {
        return extend;
    }
}