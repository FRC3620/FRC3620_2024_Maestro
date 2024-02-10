// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class SolidPattern extends Pattern {
    public LightSegment lightSegment=new LightSegment( 0, 0);
    private Color color;

    public SolidPattern setColor(Color color){
        this.color=color;
        return this;
    }

    @Override
    public void start (LightSegment lightSegment) {}

    @Override
    public void periodic(LightSegment lightSegment) {    
        RobotContainer.blinkySubsystem.updateLEDS(lightSegment.first, lightSegment.last,color);
    }
}
