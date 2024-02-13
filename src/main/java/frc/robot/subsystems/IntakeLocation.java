// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


/** Add your docs here. */
public class IntakeLocation {
    public static IntakeLocation testLocation1 = new IntakeLocation(0, 0, 0);
    public static IntakeLocation testLocation2 = new IntakeLocation(1, 1, 1);
    double shoulder;
    /**
     * This is how far we want the intake to extend in centimeters
     */
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