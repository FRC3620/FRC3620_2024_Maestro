// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


/** Add your docs here. */
public class IntakeLocation {
    public static IntakeLocation IntakeIn = new IntakeLocation(-17.6);
    public static IntakeLocation IntakeOut = new IntakeLocation(5);
   

    Double shoulder;

    public IntakeLocation(double shoulder) {
        this.shoulder = shoulder;
    
    }

    public Double getShoulder() {
        return shoulder;
    }

    @Override
    public String toString() {
        return "IntakeLocation [shoulder=" + shoulder +"]";
    }

}