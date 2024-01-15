package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface INavigationSubsystem {
    public default void reset() { }


    /**
     * 
     * @return return heading from gyro in degrees, increasing angle is clockwise.
     */
    public double getRawHeading();

    /**
     * 
     * @return return heading in degress; 0 is pointed away from our
     * driver, increasing angle is clockwise, and limit to -180 to 180.
     */
    public default double getCorrectedHeading() {
        double angle = getRawHeading() + getHeadingOffset();

        angle = angle % 360;
		
		if (angle > 180){
			angle = -360 + angle;
		}
		if (angle < -180){
			angle = 360 + angle;
		}
		
		return angle;
    }

    /**
     * 
     * @return
     */
    public default double getHeadingOffset() {
        return 0;
    }

    /**
     * Set the difference between the raw heading and pointing to the opposite team
     * driver stations. Use this when we aren't pointed that way at the beginning of a match.
     * @param headingOffset in degrees
     */
    public default void setHeadingOffset(double headingOffset) {
    }

    /**
     * return the Rotation2d for which way we are facing. Expressed as a 
     * Rotation2d, increasing values are CCW, and 0 is facing the red
     * alliance wall.
     * @param alliance
     * @return
     */
    public default Rotation2d getOdometryHeading(Alliance alliance) {
        // 0 is pointed at opponent's driver stations, increasing value is clockwise, in degrees
        double headingInDegrees = getCorrectedHeading();
        // 0 is pointed at opponent's driver stations, increasing value is CCW, in radians
        double trigClassHeadingInRadians = -Units.degreesToRadians(headingInDegrees);
        if (alliance == Alliance.Red) {
            trigClassHeadingInRadians += Math.PI;
        }
        if (trigClassHeadingInRadians > Math.PI) trigClassHeadingInRadians -= (2 * Math.PI);
        // 0 is pointed at RED alliance driver station
        return new Rotation2d(trigClassHeadingInRadians);
    }

    public double getPitch();

    public double getRoll();
    
}
