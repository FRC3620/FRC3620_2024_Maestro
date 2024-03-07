// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


/** Add your docs here. */
public class IntakeLocation {
    public static IntakeLocation groundPosition = new IntakeLocation(-17.6,12, 5);
    public static IntakeLocation midGroundPositionForPickup = new IntakeLocation(5,12, 0);
    public static IntakeLocation midGroundPositionForHome = new IntakeLocation(18,12, 0);
    public static IntakeLocation homePosition = new IntakeLocation(1.84, 0, 0);//0, 0, 5.71);
    public static IntakeLocation ampPosition = new IntakeLocation(72.1, 11.2, 22.5);
    public static IntakeLocation trapPosition = new IntakeLocation(69.342, 17.962, 24.976);
    public static IntakeLocation preclimbPosition = new IntakeLocation(70, 0, 0);
    public static IntakeLocation parkPosition = new IntakeLocation(null, null, null);

    Double shoulder, extend, wrist;

    public IntakeLocation(Double shoulder, Double extend, Double wrist) {
        this.shoulder = shoulder;
        this.extend = extend;
        this.wrist = wrist;
    }

    public IntakeLocation(double shoulder, double extend, double wrist) {
        this.shoulder = shoulder;
        this.extend = extend;
        this.wrist = wrist;
    }

    public Double getShoulder() {
        return shoulder;
    }

    public Double getWrist() {
        return wrist;
    }

    public Double getExtend() {
        return extend;
    }

    @Override
    public String toString() {
        return "IntakeLocation [shoulder=" + shoulder + ", extend=" + extend + ", wrist=" + wrist + "]";
    }

}