// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class LightSegment {



     int first;
    int last;

    Pattern currentPattern;
    


    public LightSegment ( int first, int last) {
    

        this.first=first;
        this.last=last;
    }

    /**
     * Define what pattern the light segment should display.
     * @param pattern
     */
    public void setPattern (Pattern pattern) {
        if (currentPattern != null) {
            currentPattern.done(this);
        }
        currentPattern = pattern;
        currentPattern.start(this);
        
    }

    public void periodic() {
        if (currentPattern != null) {
            currentPattern.periodic(this);
        }
    }

    public int getFirst() {
        return first;
    }

    public int getLast() {
        return last;
    }

    


}
