package org.usfirst.frc3620.misc;

/**
 * A container for the vectors for all four corner drive units.
 */
public class DriveVectors {
    @Override
	public String toString() {
		return "DriveVectors [leftFront=" + leftFront + ", rightFront=" + rightFront + ", leftBack=" + leftBack
				+ ", rightBack=" + rightBack + "]";
	}

	public Vector leftFront;
	public Vector rightFront;
	public Vector leftBack;
	public Vector rightBack;
}
