package org.usfirst.frc3620.misc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SwerveCalculator {
    public double halfChasisWidth;  //Chasis width and length must be in meters 
    public double halfChasisLength; //Chasis width and length must be in meters 
    public double maxDriveVelocity;

	public SwerveCalculator(double chasisWidth, double chasisLength, double maxVelocity) {		
		halfChasisWidth = chasisWidth / 2;     
        halfChasisLength = chasisLength / 2;	 	 
        maxDriveVelocity = maxVelocity;
	}

	/**
	 * For a X, Y joystick position, calculate the strafe angle.
	 * @param joyX joystick X, positive to the right
	 * @param joyY joystick Y, positive forward
	 * @return strafe angle, 0 straight ahead, 90 to the right...
	 */
	public static double calculateStrafeAngle (double joyX, double joyY) {
		return 90 - (Math.atan2(joyY, joyX)*(180/Math.PI)); //get our desired strafing angle in degrees
	}

	public static double calculateStrafeVectorMagnitude (double joyX, double joyY) {
		return Math.sqrt((joyX*joyX) + (joyY*joyY)); //get desired strafing magnitude
	}

	public DriveVectors calculateEverything (double joyX, double joyY, double joyRotation) {
        return calculateEverythingFromVector(calculateStrafeAngle(joyX, joyY), calculateStrafeVectorMagnitude(joyX, joyY), joyRotation);
	}

	/**
	 * Calculate the swerve unit vectors for a certain strafe / rotate setup.
	 * 
	 * The strafeAngle is relative to the robot; 0 is straight ahead, and increasing angle is clockwise
	 * (the way a compass works).
	 * 
	 * joyRotation is also the way a compass works (positive turns the robot clockwise).	
	 * 
	 * The DriveVectors returned have the convection that 0 is pointing to the right, and 
	 * increasing angle is CCW (the way we were taught in math class)
	 * 	 * 
	 * @param strafeVectorAngle
	 * @param strafeVectorMagnitude
	 * @param joyRotation
	 * @return
	 */
	public DriveVectors calculateEverythingFromVector (double strafeVectorAngle, double strafeVectorMagnitude, double joyRotation) {
		// these will be in math coordinates - 0 points right, increasing values CCW
		double r_strafeVectorAngle = 90.0 - strafeVectorAngle;
		double r_joyRotation = joyRotation;

		SmartDashboard.putNumber("calc.va_r", r_strafeVectorAngle);
		SmartDashboard.putNumber("calc.va", strafeVectorAngle);
		SmartDashboard.putNumber("calc.jr", joyRotation);
		SmartDashboard.putNumber("calc.jr_r", r_joyRotation);

		//calculate X and Y components of the new strafing vector
		double strafeX = strafeVectorMagnitude*Math.cos(r_strafeVectorAngle*Math.PI/180);
		double strafeY = strafeVectorMagnitude*Math.sin(r_strafeVectorAngle*Math.PI/180);

		double a = strafeX-(r_joyRotation*halfChasisLength);    //joyX is the horizontal input on the strafe joystick (the horizontal component of the desired vehicle speed), its units are in/s. 
		double b = strafeX+(r_joyRotation*halfChasisLength);	   //variables A and B are horizontal components of WHEEL velocity, their units are in/s
		double c = strafeY-(r_joyRotation*halfChasisWidth);     //joyY is the vertical input on the strafe joystick (the vertical component of the desired vehicle speed), its units are in/s.
		double d = strafeY+(r_joyRotation*halfChasisWidth);     //variables C and D are vertical components of WHEEL velocity, their units are in/s

		DriveVectors rv = new DriveVectors();    //created a DriveVectors object, which holds 4 vector objects (each with its respective direction and magnitude), one for each wheel
		rv.leftFront = fancyCalc(b,  d);          
		rv.rightFront = fancyCalc(b,  c);        //Each wheel velocity vector is calculated using one X (horizontal) component and one Y (vertical) component
		rv.leftBack = fancyCalc(a,  d);          //see fancyCalc();
		rv.rightBack = fancyCalc(a,  c);
		return rv;
	}

	Vector fancyCalc (double x_r, double y_r) {             //takes in an X and a Y component of a vector and returns a new Vector object with: 
		double angle = Math.atan2(y_r, x_r)*(180/Math.PI);  //direction (in degrees) 
		double velocity = Math.sqrt((x_r*x_r) + (y_r*y_r)); //speed (in meters per second)
		if(velocity>maxDriveVelocity) {
            velocity = maxDriveVelocity;
        }
		return new Vector (angle, velocity);
	}

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

	public static double calculateAngleDifference(double y, double x) {//x = target angle, y = source angle, this finds the shortest angle difference for the swerve modules
		double diff = x - y;

		diff = normalizeAngle(diff);

		return diff;
	}
	
	public static Vector determineNewVector (Vector calculated, Vector current, double steeringCutoff) { //takes 2 vectors and calculates the angle between them and returns a new "fixed" vector
		double diff = calculateAngleDifference(current.getDirection(), calculated.getDirection());
		if(Math.abs(diff) < 90 ) { // if the difference is greater than 90 degrees, the angle is normalized (see normalizeAngle()) and added 180 degrees 
			if(Math.abs(calculated.getMagnitude())>= steeringCutoff){
				return new Vector(current.getDirection() + diff, calculated.getMagnitude());     //this also means that it is faster to invert the power on the motors and go to the angle 180 greater than the one returned by fancyCalc
			}
			else{
				return new Vector(current.getDirection(), 0);
			}
		}
		else {
			if(diff>0) {
				if(Math.abs(calculated.getMagnitude())>= steeringCutoff){
					return new Vector(current.getDirection() + diff - 180, -calculated.getMagnitude());     //this also means that it is faster to invert the power on the motors and go to the angle 180 greater than the one returned by fancyCalc
				}
				else{
					return new Vector(current.getDirection(), 0);
				}
			}
			else {
				if(Math.abs(calculated.getMagnitude())>= steeringCutoff){
					return new Vector(current.getDirection() + diff + 180, -calculated.getMagnitude());     //this also means that it is faster to invert the power on the motors and go to the angle 180 greater than the one returned by fancyCalc
				}
				else{
					return new Vector(current.getDirection(), 0);
				}
			}  //new vector includes the new "fixed" angle and the motor inverted only if the angle difference is greater than 90 degrees
		}
	}
	
	public static DriveVectors fixVectors(DriveVectors calculatedVectors, DriveVectors currentVectors ) { //fixes every vector of the DriverVectors object and returns them
		DriveVectors fixedVectors = new DriveVectors();
		
		fixedVectors.leftFront = determineNewVector(calculatedVectors.leftFront, currentVectors.leftFront, 20);
		fixedVectors.rightFront = determineNewVector(calculatedVectors.rightFront, currentVectors.rightFront, 20);
		fixedVectors.leftBack = determineNewVector(calculatedVectors.leftBack, currentVectors.leftBack, 20);
		fixedVectors.rightBack = determineNewVector(calculatedVectors.rightBack, currentVectors.rightBack, 20);
		
		//System.out.println ("calculated: " + calculatedVectors.leftBack);
		//System.out.println ("new: " + fixedVectors.leftBack);
		
		return fixedVectors;
	}
	public static DriveVectors fixVectors(DriveVectors calculatedVectors, DriveVectors currentVectors, double cutoff ) { //fixes every vector of the DriverVectors object and returns them
		DriveVectors fixedVectors = new DriveVectors();
		
		fixedVectors.leftFront = determineNewVector(calculatedVectors.leftFront, currentVectors.leftFront, cutoff);
		fixedVectors.rightFront = determineNewVector(calculatedVectors.rightFront, currentVectors.rightFront, cutoff);
		fixedVectors.leftBack = determineNewVector(calculatedVectors.leftBack, currentVectors.leftBack, cutoff);
		fixedVectors.rightBack = determineNewVector(calculatedVectors.rightBack, currentVectors.rightBack, cutoff);
		
		//System.out.println ("calculated: " + calculatedVectors.leftBack);
		//System.out.println ("new: " + fixedVectors.leftBack);
		
		return fixedVectors;
	}
}
