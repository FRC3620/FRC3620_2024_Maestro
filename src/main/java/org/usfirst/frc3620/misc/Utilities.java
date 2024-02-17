// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

import java.util.function.Consumer;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.SendableRegistry.CallbackData;

/** Add your docs here. */
public class Utilities {
	static Logger logger = EventLogging.getLogger(Utilities.class, Level.INFO);
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

	public static void dumpSendables(String label, String subsystemName) {
		logger.info ("Dumping Sendables: {}", label);
		SendableRegistry.foreachLiveWindow(0, new Callback(subsystemName));
	}

	static class Callback implements Consumer<SendableRegistry.CallbackData> {
		String subsystemName;
		Callback (String s) {
			this.subsystemName = s;
		}
		@Override
		public void accept(CallbackData t) {
			if (subsystemName == null || subsystemName.equals(t.subsystem)) {
				logger.info ("- {} -> {}", t.subsystem, t.name);
			}
		}
	}

}
