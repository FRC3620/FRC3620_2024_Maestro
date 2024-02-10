// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class BlinkPattern extends Pattern{
  //  private int onSec;
    private double Sec;
    private Color color;
    private Timer timer;
    private boolean lSate;

    public BlinkPattern(){
        timer=new Timer();
        timer.reset();
        timer.start();
    }
    public BlinkPattern setColor(Color color){
        this.color=color;
        return this;
    }
    public BlinkPattern setBlink(double Sec){
        this.Sec=Sec;
        // this.onSec=onSec;
        return this;
    }

    public void start(LightSegment lightSegment){}

    public void periodic(LightSegment lightSegment){
            if( timer.hasElapsed(Sec)){
                if (!lSate) {
                 lSate=true;
            RobotContainer.blinkySubsystem.updateLEDS(lightSegment.first, lightSegment.last, Color.kBlack);   
                    timer.restart();
                }else if (lSate) {
                    lSate=false;
        RobotContainer.blinkySubsystem.updateLEDS(lightSegment.first, lightSegment.last,color);
                    timer.restart();
                }
    }  
        
    }
    }
