// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

/** Add your docs here. */
public class Utilities {
    /**
	 * This method makes sure the angle difference calculated falls between -180 degrees and 180 degrees
	 * @param angle
	 * @return
	 */
	public static double normalizeAngle(double angle) {
		angle = angle % 360;
		
		if(angle > 180){
			angle = -360 + angle;
		}
		if(angle <= -180){
			angle = 360 + angle;
		}
		
		if(angle == -0) {angle = 0;}
		
		return angle;
	}

}
