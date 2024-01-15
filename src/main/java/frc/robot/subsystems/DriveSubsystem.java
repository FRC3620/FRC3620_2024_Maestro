package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import java.util.function.Supplier;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.TeleOpDriveCommand;

import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.DriveVectors;
import org.usfirst.frc3620.misc.MotorSetup;
import org.usfirst.frc3620.misc.SwerveParameters;
import org.usfirst.frc3620.misc.SwerveCalculator;
import org.usfirst.frc3620.misc.Vector;

public class DriveSubsystem extends SubsystemBase implements Supplier<SwerveModulePosition[]> {
	private final double DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT = 0.3;
	private final double AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT = 0.3;

	// encoder gives 360 degrees per encoder tick, and is geared 18:1 to the wheel.
	private final double AZIMUTH_ENCODER_CONVERSION_FACTOR = 360. / 18.; // (1/(11.7))*235; 
	private final double SPEED_ENCODER_TICS = 42;
	//private final double WHEEL_TO_ENCODER_RATIO_VELOCITY = (1/8.31); //for every full wheel turn, the motor turns 8.31 times
	private final double WHEEL_RADIUS = 2; //in inches
	private final double MAX_VELOCITY_RPM = 1200; //maximum velocity that the robot will travel when joystick is at full throtle, measured in RPM orignally 750
	private final double MAX_TURN = 6; //maximum angular velocity at which the robot will turn when joystick is at full throtle, measured in rad/s

	Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

	INavigationSubsystem navigationSubsystem;

	SwerveModulePosition[] swerveModulePositions;

	private boolean logSpinTransitions = false;
	private boolean putDriveVectorsInNetworkTables = false;

	public CANSparkMaxSendable rightFrontDrive;
	public CANSparkMaxSendable rightFrontAzimuth;
	public RelativeEncoder rightFrontDriveEncoder;
	public RelativeEncoder rightFrontAzimuthEncoder;
	public AnalogInput rightFrontHomeEncoder;
	public SparkPIDController rightFrontVelPID; // don't assign unless we have the motor controller
	public SparkPIDController rightFrontPositionPID;

	public CANSparkMaxSendable leftFrontDrive;
	public CANSparkMaxSendable leftFrontAzimuth;
	public RelativeEncoder leftFrontDriveEncoder;
	public RelativeEncoder leftFrontAzimuthEncoder;
	public AnalogInput leftFrontHomeEncoder;
	public SparkPIDController leftFrontVelPID;
	public SparkPIDController leftFrontPositionPID;

	public CANSparkMaxSendable leftBackDrive;
	public CANSparkMaxSendable leftBackAzimuth;
	public RelativeEncoder leftBackDriveEncoder;
	public RelativeEncoder leftBackAzimuthEncoder;
	public AnalogInput leftBackHomeEncoder;
	public SparkPIDController leftBackVelPID;
	public SparkPIDController leftBackPositionPID;

	public CANSparkMaxSendable rightBackDrive;
	public CANSparkMaxSendable rightBackAzimuth;
	public RelativeEncoder rightBackDriveEncoder;
	public RelativeEncoder rightBackAzimuthEncoder;
	public AnalogInput rightBackHomeEncoder;
	public SparkPIDController rightBackVelPID;
	public SparkPIDController rightBackPositionPID;

	//***********************************************************************************************************
	//                    MUST MAKE SURE THESE VALUES ARE RIGHT BEFORE RUNNING SWERVE CODE
	//***********************************************************************************************************

	// these will get overwritten from the robot_parameters.json. Putting something here so that we don't get
	// divide by zero errors if the dimensions are missing in the .json.
	private double CHASIS_WIDTH = 22.25; //inches
	private double CHASIS_LENGTH = 24.25; //inches
	
	private final double WHEEL_CIRCUMFERENCE = 2*Math.PI*WHEEL_RADIUS;
	private final double MAX_VELOCITY_IN_PER_SEC = MAX_VELOCITY_RPM*WHEEL_CIRCUMFERENCE/60; //max velocity in inches per second origanlly 60

 
	// readings of the absolute encoders when the wheels are pointed at true 0 degrees (gears to front of robot)
	private double RIGHT_FRONT_ABSOLUTE_OFFSET;
	private double LEFT_FRONT_ABSOLUTE_OFFSET;
	private double LEFT_BACK_ABSOLUTE_OFFSET;
	private double RIGHT_BACK_ABSOLUTE_OFFSET;

	private double kPositionP = 0.005;
	private double kPositionI = 0.00000;
	private double kPositionD = .0;
	private double kPositionFF = 0;
	private double kPositionIz = 0;
	private double kPositionMaxOutput = 1;
	
	private double kPositionMinOutput = -1;
	
	private double kVelocityP = 0.01;  //0.01
	private double kVelocityI = 0.000000;
	private double kVelocityD = 0.1;  //0.1
	private double kVelocityFF = 0.0;
	private double kVelocityIz = 0;
	private double kVelocityMaxOutput = 1;
	private double kVelocityMinOutput = -1;

	private boolean drivePIDTuning = false;

	private boolean fieldRelative = true;

	private PIDController spinPIDController;
	private double kSpinP = 0.015; //0.005 works
	private double kSpinI = 0.00000; //0.0000
	private double kSpinD = 0.001; //0.000
	private boolean autoSpinMode;
	private boolean forceManualMode = false;
	private double currentHeading;
	private double targetHeading;
	private double spinPower;

	//***********************************************************************************************************
	//***********************************************************************************************************

	SwerveCalculator sc;
	DriveVectors oldVectors;
	SwerveParameters swerveParameters;

  public DriveSubsystem(INavigationSubsystem navigationSubsystem) {
		this.navigationSubsystem = navigationSubsystem;

		swerveParameters = null;
		if (RobotContainer.robotParameters != null) {
			swerveParameters = RobotContainer.robotParameters.getSwerveParameters();
		}
		if (swerveParameters == null) {
			logger.error("all swerve parameters are missing");
		} else {
			String missingSwerveParameters = swerveParameters.whichSwerveParametersAreMissing();
			if (missingSwerveParameters != null) {
				logger.error("missing swerve parameters: {}", missingSwerveParameters);
			} else {
				RIGHT_BACK_ABSOLUTE_OFFSET = swerveParameters.getRightBackAbsoluteOffset();
				LEFT_BACK_ABSOLUTE_OFFSET = swerveParameters.getLeftBackAbsoluteOffset();
				RIGHT_FRONT_ABSOLUTE_OFFSET = swerveParameters.getRightFrontAbsoluteOffset();
				LEFT_FRONT_ABSOLUTE_OFFSET = swerveParameters.getLeftFrontAbsoluteOffset();
				CHASIS_LENGTH = swerveParameters.getChassisLength();
				CHASIS_WIDTH = swerveParameters.getChassisWidth();
			}
		}
		sc = new SwerveCalculator(CHASIS_WIDTH, CHASIS_LENGTH, MAX_VELOCITY_IN_PER_SEC);

		setupMotorsAndEncoders();

		SmartDashboard.putNumber("drive.position_pid.p", kPositionP);
		SmartDashboard.putNumber("drive.position_pid.i", kPositionI);
		SmartDashboard.putNumber("drive.position_pid.d", kPositionD);
		SmartDashboard.putNumber("drive.position_pid.iz", kPositionIz);
		SmartDashboard.putNumber("drive.position_pid.ff", kPositionFF);
		SmartDashboard.putNumber("drive.position_pid.max", kVelocityMaxOutput);
		SmartDashboard.putNumber("drive.position_pid.min", kVelocityMinOutput);
		
		SmartDashboard.putNumber("drive.velocity_pid.p", kVelocityP);
		SmartDashboard.putNumber("drive.velocity_pid.i", kVelocityI);
		SmartDashboard.putNumber("drive.velocity_pid.d", kVelocityD);
		SmartDashboard.putNumber("drive.velocity_pid.iz", kVelocityIz);
		SmartDashboard.putNumber("drive.velocity_pid.ff", kVelocityFF);
		SmartDashboard.putNumber("drive.velocity_pid.max", kVelocityMaxOutput);
		SmartDashboard.putNumber("drive.velocity_pid.min", kVelocityMinOutput);

		SmartDashboard.putBoolean("drive.are_we_tuning_drive_pid", drivePIDTuning);

		this.setDefaultCommand(new TeleOpDriveCommand(this));
		
		spinPIDController = new PIDController(kSpinP, kSpinI, kSpinD);
		spinPIDController.enableContinuousInput(-180, 180); //sets a circular range instead of a linear one. 
		spinPIDController.setTolerance(3);
		addChild("Spin PID", spinPIDController);

		fixRelativeEncoders();

		swerveModulePositions = new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition()
		};
		fillSwerveModulePositions();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("drive.rf.home_encoder", getHomeEncoderHeading(rightFrontHomeEncoder));
		SmartDashboard.putNumber("drive.lf.home_encoder", getHomeEncoderHeading(leftFrontHomeEncoder));
		SmartDashboard.putNumber("drive.lb.home_encoder", getHomeEncoderHeading(leftBackHomeEncoder));
		SmartDashboard.putNumber("drive.rb.home_encoder", getHomeEncoderHeading(rightBackHomeEncoder));

		SmartDashboard.putNumber("drive.rf.encoder_diff", encoderDifference(Corner.RF));
		SmartDashboard.putNumber("drive.lf.encoder_diff", encoderDifference(Corner.LF));
		SmartDashboard.putNumber("drive.lb.encoder_diff", encoderDifference(Corner.LB));
		SmartDashboard.putNumber("drive.rb.encoder_diff", encoderDifference(Corner.RB));

		if (rightFrontDriveEncoder != null) {
			SmartDashboard.putNumber("drive.rf.drive.velocity", rightFrontDriveEncoder.getVelocity());
			SmartDashboard.putNumber("drive.rf.drive.position", rightFrontDriveEncoder.getPosition());
			SmartDashboard.putNumber("drive.rf.azimuth.position_raw", rightFrontAzimuthEncoder.getPosition());
			SmartDashboard.putNumber("drive.rf.azimuth.velocity", rightFrontAzimuthEncoder.getVelocity());
			SmartDashboard.putNumber("drive.rf.azimuth.position", getFixedPosition(rightFrontAzimuthEncoder));
			SmartDashboard.putNumber("drive.rf.drive.current", rightFrontDrive.getOutputCurrent());
			SmartDashboard.putNumber("drive.rf.drive.power", rightFrontDrive.getAppliedOutput());
		}
		if (leftFrontDriveEncoder != null) {
			SmartDashboard.putNumber("drive.lf.drive.velocity", leftFrontDriveEncoder.getVelocity());
			SmartDashboard.putNumber("drive.lf.azimuth.position_raw", leftFrontAzimuthEncoder.getPosition());
			SmartDashboard.putNumber("drive.lf.azimuth.velocity", leftFrontAzimuthEncoder.getVelocity());
			SmartDashboard.putNumber("drive.lf.azimuth.position", getFixedPosition(leftFrontAzimuthEncoder));
			SmartDashboard.putNumber("drive.lf.drive.current", leftFrontDrive.getOutputCurrent());
			SmartDashboard.putNumber("drive.lf.drive.power", leftFrontDrive.getAppliedOutput());
		}
		if (leftBackDriveEncoder != null) {
			SmartDashboard.putNumber("drive.lb.drive.velocity", leftBackDriveEncoder.getVelocity());
			SmartDashboard.putNumber("drive.lb.azimuth.position_raw", leftBackAzimuthEncoder.getPosition());
			SmartDashboard.putNumber("drive.lb.azimuth.velocity", leftBackAzimuthEncoder.getVelocity());
			SmartDashboard.putNumber("drive.lb.azimuth.position", getFixedPosition(leftBackAzimuthEncoder));
			SmartDashboard.putNumber("drive.lb.drive.current", leftBackDrive.getOutputCurrent());
			SmartDashboard.putNumber("drive.lb.drive.power", leftBackDrive.getAppliedOutput());
		}
		if (rightBackDriveEncoder != null) {
			SmartDashboard.putNumber("drive.rb.drive.velocity", rightBackDriveEncoder.getVelocity());
			SmartDashboard.putNumber("drive.rb.azimuth.position_raw", rightBackAzimuthEncoder.getPosition());
			SmartDashboard.putNumber("drive.rb.azimuth.velocity", rightBackAzimuthEncoder.getVelocity());
			SmartDashboard.putNumber("drive.rb.azimuth.position", getFixedPosition(rightBackAzimuthEncoder));
			SmartDashboard.putNumber("drive.rb.drive.current", rightBackDrive.getOutputCurrent());
			SmartDashboard.putNumber("drive.rb.drive.power", rightBackDrive.getAppliedOutput());
		}

		SmartDashboard.putBoolean("drive.are_we_stopped", areWeStopped());

		if (rightFrontDrive  != null) {
			updateVelocityPIDs(rightFrontVelPID, leftFrontVelPID, leftBackVelPID, rightBackVelPID);
		}

		drivePIDTuning = SmartDashboard.getBoolean("drive.are_we_tuning_drive_pid", false);

		currentHeading = navigationSubsystem.getCorrectedHeading();

		double commandedSpin = RobotContainer.getDriveSpinJoystick();

		if(forceManualMode){
			setManualSpinMode();
		}
		else{
			if(Math.abs(commandedSpin) != 0){
				setManualSpinMode();
			}else{
				setAutoSpinMode();
			}
		}

		if(!autoSpinMode){
			periodicManualSpinMode();
		}else{
			periodicAutoSpinMode();
		}
		SmartDashboard.putNumber("drive.target_heading", targetHeading);
		SmartDashboard.putNumber("drive.spin_power", spinPower);

		SmartDashboard.putBoolean("drive.auto_spin_mode", autoSpinMode);
		SmartDashboard.putBoolean("drive.field_relative", fieldRelative);

		SmartDashboard.putNumber("driver.joy.x", RobotContainer.getDriveHorizontalJoystick());
		SmartDashboard.putNumber("driver.joy.y", RobotContainer.getDriveVerticalJoystick());
		SmartDashboard.putNumber("driver.joy.spin", RobotContainer.getDriveSpinJoystick());

		fillSwerveModulePositions();
	}

	public SwerveModulePosition[] get() {
		return swerveModulePositions;
	}

	void fillSwerveModulePositions() {
		fillSwerveModulePosition(0, leftFrontDriveEncoder, leftFrontAzimuthEncoder);
		fillSwerveModulePosition(1, rightFrontDriveEncoder, rightFrontAzimuthEncoder);
		fillSwerveModulePosition(2, leftBackDriveEncoder, leftBackAzimuthEncoder);
		fillSwerveModulePosition(3, rightBackDriveEncoder, rightBackAzimuthEncoder);
	}

	void fillSwerveModulePosition(int index, RelativeEncoder driveEncoder, RelativeEncoder azimuthEncoder) {
		double distanceInInches = 0;
		if (driveEncoder != null) distanceInInches = driveEncoder.getPosition();
		// I don't know why we have to negate this, but we do
		double distanceInMeters = - Units.inchesToMeters(distanceInInches);
		swerveModulePositions[index].distanceMeters = distanceInMeters;

		double azimuthInDegrees0ToRight = getFixedPosition(azimuthEncoder);
		double azimuthInRadians0InFront = Units.degreesToRadians(azimuthInDegrees0ToRight + 90);
		swerveModulePositions[index].angle = new Rotation2d(azimuthInRadians0InFront);
	}

  	public void periodicManualSpinMode(){
		setTargetHeading(currentHeading);
		double commandedSpin = RobotContainer.getDriveSpinJoystick();
		spinPower = commandedSpin;
  	}

  	public void periodicAutoSpinMode(){
		spinPIDController.setSetpoint(targetHeading);
		spinPower = spinPIDController.calculate(currentHeading);

		SmartDashboard.putNumber("drive.spin_pid_error", spinPIDController.getPositionError());
		SmartDashboard.putNumber("drive.spin_pid_setpoint", spinPIDController.getSetpoint());
  	}

  	public double getStrafeXValue() {
		double rv = MAX_VELOCITY_RPM*RobotContainer.getDriveHorizontalJoystick();
		return rv;
	}

	public double getStrafeYValue() {
		double rv = MAX_VELOCITY_RPM*RobotContainer.getDriveVerticalJoystick();
		return rv;
	}

	public double getRotationValue() {
		double rv = MAX_TURN*RobotContainer.getDriveSpinJoystick();
		return rv;
	}

	public void testDrive(double azimuthPower, double drivePower){
		if (rightFrontDrive != null) {
			rightFrontDrive.set(drivePower);
			leftFrontDrive.set(drivePower);
			rightBackDrive.set(drivePower);
			leftBackDrive.set(drivePower);

			rightFrontAzimuth.set(azimuthPower);
			leftFrontAzimuth.set(azimuthPower);
			rightBackAzimuth.set(azimuthPower);
			leftBackAzimuth.set(azimuthPower);
		}
	}

	public void stopDrive(){
		if (rightFrontDrive != null) {
			rightFrontDrive.set(0);
			leftFrontDrive.set(0);
			rightBackDrive.set(0);
			leftBackDrive.set(0);

			rightFrontAzimuth.set(0);
			leftFrontAzimuth.set(0);
			rightBackAzimuth.set(0);
			leftBackAzimuth.set(0);
		}
	}

	public void azimuthTest(double heading){
		DriveVectors newVectors = new DriveVectors();
		newVectors.leftFront = new Vector(heading, 0);         
		newVectors.rightFront = new Vector(heading, 0);      
		newVectors.leftBack = new Vector(heading, 0);        
		newVectors.rightBack = new Vector(heading, 0);

		DriveVectors currentDirections = getCurrentVectors();

		newVectors = SwerveCalculator.fixVectors(newVectors, currentDirections);

		if (rightFrontDrive != null) {
			rightFrontPositionPID.setReference(newVectors.rightFront.getDirection(), ControlType.kPosition);
			leftFrontPositionPID.setReference(newVectors.leftFront.getDirection(), ControlType.kPosition);
			leftBackPositionPID.setReference(newVectors.leftBack.getDirection(), ControlType.kPosition);
			rightBackPositionPID.setReference(newVectors.rightBack.getDirection(), ControlType.kPosition);
		}
	}

	/**
	 * Calculate DriveVectors. Correct for field orientation based on the current state
	 * of the isFieldRelative.
	 * 
	 * @param vx joystick x
	 * @param vy joystick y
	 * @param vr rotate
	 * @return drive vectors for the swerve units.
	 */
	DriveVectors calculateEverything(double vx, double vy, double vr) {
		return calculateEverything(vx, vy, vr, getFieldRelative());
	}

	DriveVectors calculateEverything (double vx, double vy, double vr, boolean doFieldRelative) {
		double strafeVectorAngle = SwerveCalculator.calculateStrafeAngle(vx, vy);
		double strafeVectorMagnitude = SwerveCalculator.calculateStrafeVectorMagnitude(vx, vy);
		if (doFieldRelative) {
			double robotHeading = navigationSubsystem.getCorrectedHeading();
			 //get NavX heading in degrees (from -180 to 180)
			strafeVectorAngle = SwerveCalculator.normalizeAngle(strafeVectorAngle - robotHeading); //add heading to strafing angle to find our field-relative angle
		} else {
			// not sure this is wise!
			strafeVectorAngle = SwerveCalculator.normalizeAngle(strafeVectorAngle);
		}
		if (putDriveVectorsInNetworkTables) {
			SmartDashboard.putNumber("drive.vx", vx);
			SmartDashboard.putNumber("drive.vy", vy);
			SmartDashboard.putNumber("drive.strafeVectorAngle", strafeVectorAngle);
			SmartDashboard.putNumber("drive.strafeVectorAngle+correction", strafeVectorAngle);
			SmartDashboard.putNumber("drive.strafeVectorMagnitude", strafeVectorMagnitude);
		}

		return sc.calculateEverythingFromVector(strafeVectorAngle, strafeVectorMagnitude, vr);
	}

	public void teleOpDrive(double strafeX, double strafeY, double spinX) {
		double vx = strafeX*MAX_VELOCITY_IN_PER_SEC;
		double vy = strafeY*MAX_VELOCITY_IN_PER_SEC;
		double vr = spinX*MAX_TURN;

		DriveVectors newVectors = calculateEverything(vx, vy, vr);
		if (putDriveVectorsInNetworkTables) {
			SmartDashboard.putString("drive.teleop.dv.pre", newVectors.toString());
		}

		DriveVectors currentDirections = getCurrentVectors();

		newVectors = SwerveCalculator.fixVectors(newVectors, currentDirections);
		if (putDriveVectorsInNetworkTables) {
			SmartDashboard.putString("drive.teleop.dv.post", newVectors.toString());
		}

		if (rightFrontDrive != null) {
			rightFrontPositionPID.setReference(newVectors.rightFront.getDirection(), ControlType.kPosition);
			leftFrontPositionPID.setReference(newVectors.leftFront.getDirection(), ControlType.kPosition);
			leftBackPositionPID.setReference(newVectors.leftBack.getDirection(), ControlType.kPosition);
			rightBackPositionPID.setReference(newVectors.rightBack.getDirection(), ControlType.kPosition);

			rightFrontVelPID.setReference(newVectors.rightFront.getMagnitude(), ControlType.kVelocity);
			leftFrontVelPID.setReference(newVectors.leftFront.getMagnitude(), ControlType.kVelocity);
			leftBackVelPID.setReference(newVectors.leftBack.getMagnitude(), ControlType.kVelocity);
			rightBackVelPID.setReference(newVectors.rightBack.getMagnitude(), ControlType.kVelocity);
		}
		
		if (drivePIDTuning){
			double lbCommandedVel = newVectors.leftBack.getMagnitude();
			double lbCurrentVel = currentDirections.leftBack.getMagnitude();
			double lbVelError = lbCommandedVel - lbCurrentVel;
			SmartDashboard.putNumber("drive.lb.drive.velocity_requested", lbCommandedVel);
			SmartDashboard.putNumber("drive.lb.drive.velocity_error", lbVelError);

			double lbCommandedAzimuth = newVectors.leftBack.getDirection();
			double lbCurrentAzimuth = currentDirections.leftBack.getDirection();
			double lbAzimuthError = lbCommandedAzimuth - lbCurrentAzimuth;
			SmartDashboard.putNumber("drive.lb.azimuth.position_requested", lbCommandedAzimuth);
			SmartDashboard.putNumber("drive.lb.azimuth.position_error", lbAzimuthError);

			double lfCommandedVel = newVectors.leftFront.getMagnitude();
			double lfCurrentVel = currentDirections.leftFront.getMagnitude();
			double lfVelError = lfCommandedVel - lfCurrentVel;
			SmartDashboard.putNumber("drive.lf.drive.velocity_requested", lfCommandedVel);
			SmartDashboard.putNumber("drive.lf.drive.velocity_error", lfVelError);

			double lfCommandedAzimuth = newVectors.leftFront.getDirection();
			double lfCurrentAzimuth = currentDirections.leftFront.getDirection();
			double lfAzimuthError = lfCommandedAzimuth - lfCurrentAzimuth;
			SmartDashboard.putNumber("drive.lf.azimuth.position_requested", lfCommandedAzimuth);
			SmartDashboard.putNumber("drive.lf.azimuth.position_error", lfAzimuthError);

			double rbCommandedVel = newVectors.rightBack.getMagnitude();
			double rbCurrentVel = currentDirections.rightBack.getMagnitude();
			double rbVelError = rbCommandedVel - rbCurrentVel;
			SmartDashboard.putNumber("drive.rb.drive.velocity_requested", rbCommandedVel);
			SmartDashboard.putNumber("drive.rb.drive.velocity_error", rbVelError);

			double rbCommandedAzimuth = newVectors.rightBack.getDirection();
			double rbCurrentAzimuth = currentDirections.rightBack.getDirection();
			double rbAzimuthError = rbCommandedAzimuth - rbCurrentAzimuth;
			SmartDashboard.putNumber("drive.rb.azimuth.position_requested", rbCommandedAzimuth);
			SmartDashboard.putNumber("drive.rb.azimuth.position_error", rbAzimuthError);

			double rfCommandedVel = newVectors.rightFront.getMagnitude();
			double rfCurrentVel = currentDirections.rightFront.getMagnitude();
			double rfVelError = rfCommandedVel - rfCurrentVel;
			SmartDashboard.putNumber("drive.rf.drive.velocity_requested", rfCommandedVel);
			SmartDashboard.putNumber("drive.rf.drive.velocity_error", rfVelError);

			double rfCommandedAzimuth = newVectors.rightFront.getDirection();
			double rfCurrentAzimuth = currentDirections.rightFront.getDirection();
			double rfAzimuthError = rfCommandedAzimuth - rfCurrentAzimuth;
			SmartDashboard.putNumber("drive.rf.azimuth.position_requested", rfCommandedAzimuth);
			SmartDashboard.putNumber("drive.rf.azimuth.position_error", rfAzimuthError);
		}
	}

	/**
	 * Drive the robot in autonomous.
	 * @param heading direction to drive in, compass-wise (0 straight, +90 to the right)
	 * @param speed 0..1
	 * @param spin direction and speed to spin. -1..+1, -1 is full to left, +1 is full to right.
	 */
	public void autoDrive(double heading, double speed, double spin) {
		DriveVectors newVectors = sc.calculateEverythingFromVector(heading, MAX_VELOCITY_IN_PER_SEC*speed, spin*MAX_TURN);
		if (putDriveVectorsInNetworkTables) {
			SmartDashboard.putString("drive.auto.dv.pre", newVectors.toString());
		}

		DriveVectors currentDirections = getCurrentVectors();

		newVectors = SwerveCalculator.fixVectors(newVectors, currentDirections, 0);
		if (putDriveVectorsInNetworkTables) {
			SmartDashboard.putString("drive.auto.dv.post", newVectors.toString());
		}

		if (rightFrontDrive != null) {
			rightFrontPositionPID.setReference(newVectors.rightFront.getDirection(), ControlType.kPosition);
			leftFrontPositionPID.setReference(newVectors.leftFront.getDirection(), ControlType.kPosition);
			leftBackPositionPID.setReference(newVectors.leftBack.getDirection(), ControlType.kPosition);
			rightBackPositionPID.setReference(newVectors.rightBack.getDirection(), ControlType.kPosition);
			
			rightFrontVelPID.setReference(newVectors.rightFront.getMagnitude(), ControlType.kVelocity);
			leftFrontVelPID.setReference(newVectors.leftFront.getMagnitude(), ControlType.kVelocity);
			leftBackVelPID.setReference(newVectors.leftBack.getMagnitude(), ControlType.kVelocity);
			rightBackVelPID.setReference(newVectors.rightBack.getMagnitude(), ControlType.kVelocity);
		}
	}

	/**
	 * Set the wheels to point in a direction to strafe, but do not move the robot.
	 * @param strafeAngle degrees are from -180 to 180 degrees with 0 degrees pointing to the robot's right
	 */
	public void setWheelsToStrafe(double strafeAngle){
		// need to have non-zero velocity so that fixVectors actually changes azimuth
		double vx = Math.cos(Math.toRadians(strafeAngle))*MAX_VELOCITY_IN_PER_SEC;
		double vy = Math.sin(Math.toRadians(strafeAngle))*MAX_VELOCITY_IN_PER_SEC;
		double vr = 0;

		DriveVectors newVectors = calculateEverything(vx, vy, vr);

		DriveVectors currentDirections = getCurrentVectors();

		newVectors = SwerveCalculator.fixVectors(newVectors, currentDirections);

		if (rightFrontDrive != null) {
			rightFrontPositionPID.setReference(newVectors.rightFront.getDirection(), ControlType.kPosition);
			leftFrontPositionPID.setReference(newVectors.leftFront.getDirection(), ControlType.kPosition);
			leftBackPositionPID.setReference(newVectors.leftBack.getDirection(), ControlType.kPosition);
			rightBackPositionPID.setReference(newVectors.rightBack.getDirection(), ControlType.kPosition);
			
			// ignore velocity component of vectors
			rightFrontVelPID.setReference(0, ControlType.kVelocity);
			leftFrontVelPID.setReference(0, ControlType.kVelocity);
			leftBackVelPID.setReference(0, ControlType.kVelocity);
			rightBackVelPID.setReference(0, ControlType.kVelocity);
		}
	}
	
	/**
	 * Strafe sideways
	 * @param speed positive is to the right
	 */
	public void strafeSideways(double speed){ 
		// these angles are angles for Vectors. Math class degress:
		// 0 degrees is to the right, 90 degrees is front, -90 degrees is behind, +/-180 degrees is left

		double leftFrontAngle = 0;
		double rightFrontAngle = 0;
		double leftBackAngle = 0;
		double rightBackAngle = 0; //should be pointing forward
		double turnSpeed = speed*MAX_VELOCITY_IN_PER_SEC;

		DriveVectors currentDirections = getCurrentVectors();
		 
		DriveVectors newVectors = new DriveVectors();

		// need to have non-zero velocity so that fixVectors actually changes azimuth.
		newVectors.leftFront = new Vector (leftFrontAngle, turnSpeed);
		newVectors.rightFront = new Vector(rightFrontAngle, turnSpeed);
		newVectors.leftBack = new Vector(leftBackAngle, turnSpeed);
		newVectors.rightBack = new Vector(rightBackAngle, turnSpeed);

		newVectors = SwerveCalculator.fixVectors(newVectors, currentDirections); //gets quickest wheel angle and direction configuration
		
		if (rightFrontDrive != null) {
			rightFrontPositionPID.setReference(newVectors.rightFront.getDirection(), ControlType.kPosition);
			leftFrontPositionPID.setReference(newVectors.leftFront.getDirection(), ControlType.kPosition);
			leftBackPositionPID.setReference(newVectors.leftBack.getDirection(), ControlType.kPosition);
			rightBackPositionPID.setReference(newVectors.rightBack.getDirection(), ControlType.kPosition);
			
			rightFrontVelPID.setReference(newVectors.rightFront.getMagnitude(), ControlType.kVelocity);
			leftFrontVelPID.setReference(newVectors.leftFront.getMagnitude(), ControlType.kVelocity);
			leftBackVelPID.setReference(newVectors.leftBack.getMagnitude(), ControlType.kVelocity);
			rightBackVelPID.setReference(newVectors.rightBack.getMagnitude(), ControlType.kVelocity);

		}
	}

	public void twoWheelRotation(double speed){ 
		// these angles are angles for Vectors. Math class degress:
		// 0 degrees is to the right, 90 degrees is front, -90 degrees is behind, +/-180 degrees is left

		double leftFrontAngle = -160;
		double rightFrontAngle = 160;
		double leftBackAngle = 90; //should be pointing forward
		double rightBackAngle = 90; //should be pointing forward
		double turnSpeed = speed*MAX_VELOCITY_IN_PER_SEC;

		DriveVectors currentDirections = getCurrentVectors();
		 
		DriveVectors newVectors = new DriveVectors();

		// need to have non-zero velocity so that fixVectors actually changes azimuth.
		newVectors.leftFront = new Vector (leftFrontAngle, turnSpeed);
		newVectors.rightFront = new Vector(rightFrontAngle, turnSpeed);
		newVectors.leftBack = new Vector(leftBackAngle, turnSpeed);  // we will fix the velocity for rear below
		newVectors.rightBack = new Vector(rightBackAngle, turnSpeed);

		newVectors = SwerveCalculator.fixVectors(newVectors, currentDirections); //gets quickest wheel angle and direction configuration
		
		if (rightFrontDrive != null) {
			rightFrontPositionPID.setReference(newVectors.rightFront.getDirection(), ControlType.kPosition);
			leftFrontPositionPID.setReference(newVectors.leftFront.getDirection(), ControlType.kPosition);
			leftBackPositionPID.setReference(newVectors.leftBack.getDirection(), ControlType.kPosition);
			rightBackPositionPID.setReference(newVectors.rightBack.getDirection(), ControlType.kPosition);
			
			rightFrontVelPID.setReference(newVectors.rightFront.getMagnitude(), ControlType.kVelocity);
			leftFrontVelPID.setReference(newVectors.leftFront.getMagnitude(), ControlType.kVelocity);
			leftBackVelPID.setReference(0, ControlType.kVelocity);
			rightBackVelPID.setReference(0, ControlType.kVelocity);
		}
	}

	public void xMode(){ 
		// these angles are angles for Vectors. Math class degress:
		// 0 degrees is to the right, 90 degrees is front, -90 degrees is behind, +/-180 degrees is left

		double leftFrontAngle = -45;
		double rightFrontAngle = 45;
		double leftBackAngle = 45;
		double rightBackAngle = -45;

		DriveVectors currentDirections = getCurrentVectors();
		 
		DriveVectors newVectors = new DriveVectors();

		// need to have non-zero velocity so that fixVectors actually changes azimuth.
		newVectors.leftFront = new Vector(leftFrontAngle, 20.1);
		newVectors.rightFront = new Vector(rightFrontAngle, 20.1);
		newVectors.leftBack = new Vector(leftBackAngle, 20.1);  // we will fix the velocity for rear below
		newVectors.rightBack = new Vector(rightBackAngle, 20.1);

		newVectors = SwerveCalculator.fixVectors(newVectors, currentDirections); //gets quickest wheel angle and direction configuration
		
		if (rightFrontDrive != null) {
			rightFrontPositionPID.setReference(newVectors.rightFront.getDirection(), ControlType.kPosition);
			leftFrontPositionPID.setReference(newVectors.leftFront.getDirection(), ControlType.kPosition);
			leftBackPositionPID.setReference(newVectors.leftBack.getDirection(), ControlType.kPosition);
			rightBackPositionPID.setReference(newVectors.rightBack.getDirection(), ControlType.kPosition);
		
			rightFrontVelPID.setReference(0, ControlType.kVelocity);
			leftFrontVelPID.setReference(0, ControlType.kVelocity);
			leftBackVelPID.setReference(0, ControlType.kVelocity);
			rightBackVelPID.setReference(0, ControlType.kVelocity);
		} 
	}

	public void setPositionPID(SparkPIDController pidController) {
		if (pidController != null) {
			pidController.setP(kPositionP);	
			pidController.setI(kPositionI);
			pidController.setD(kPositionD);
			pidController.setIZone(kPositionIz);
			pidController.setFF(kPositionFF);
			pidController.setOutputRange(kVelocityMinOutput, kVelocityMaxOutput);
		}
	}

	public void setVelocityPID(SparkPIDController pidController) {
		if (pidController != null) {
			pidController.setP(kVelocityP);	
			pidController.setI(kVelocityI);
			pidController.setD(kVelocityD);
			pidController.setIZone(kVelocityIz);
			pidController.setFF(kVelocityFF);
			pidController.setOutputRange(kVelocityMinOutput, kVelocityMaxOutput);
		}
	}

	public void updateVelocityPIDs(SparkPIDController... pidControllers) {
		if (pidControllers[0] != null) {
			double p = SmartDashboard.getNumber("drive.velocity_pid.p", kVelocityP);
			double i = SmartDashboard.getNumber("drive.velocity_pid.i", kVelocityI);
			double d = SmartDashboard.getNumber("drive.velocity_pid.d", kVelocityD);
			double iz = SmartDashboard.getNumber("drive.velocity_pid.iz", kVelocityIz);
			double ff = SmartDashboard.getNumber("drive.velocity_pid.ff", kVelocityFF);
			double max = SmartDashboard.getNumber("drive.velocity_pid.max", kVelocityMaxOutput);
			double min = SmartDashboard.getNumber("drive.velocity_pid.min", kVelocityMinOutput);

			if((p != kVelocityP)) {
				for (var pidController: pidControllers) {
					pidController.setP(p);
				}
				kVelocityP = p;	
			}
			if((i != kVelocityI)) {
				for (var pidController: pidControllers) {
					pidController.setI(i);
				}
				kVelocityI = i;	
			}
			if((d != kVelocityD)) {
				for (var pidController: pidControllers) {
					pidController.setD(d);
				}
				kVelocityD = d;	
			}
			if((iz != kVelocityIz)) {
				for (var pidController: pidControllers) {
					pidController.setIZone(iz);
				}
				kVelocityIz = iz;	
			}
			if((ff != kVelocityFF)) {
				for (var pidController: pidControllers) {
					pidController.setFF(ff);
				}
				kVelocityFF = ff;	
			}
			if((max != kVelocityMaxOutput) || (min != kVelocityMinOutput)) {
				for (var pidController: pidControllers) {
					pidController.setOutputRange(min, max);
				}
				kVelocityMaxOutput = max;	
				kVelocityMinOutput = min;	
			}
		}
	}

	public void updatePositionPID(SparkPIDController pidController) {
		double p = SmartDashboard.getNumber("drive.position_pid.p", kPositionP);
    	double i = SmartDashboard.getNumber("drive.position_pid.i", kPositionI);
    	double d = SmartDashboard.getNumber("drive.position_pid.d", kPositionD);
    	double iz = SmartDashboard.getNumber("drive.position_pid.iz", kPositionIz);
    	double ff = SmartDashboard.getNumber("drive.position_pid.ff", kPositionFF);
    	double max = SmartDashboard.getNumber("drive.position_pid.max", kPositionMaxOutput);
    	double min = SmartDashboard.getNumber("drive.position_pid.min", kPositionMinOutput);

		if((p != kPositionP)) {
			pidController.setP(p);
			kPositionP = p;	
		}
		if((i != kPositionI)) {
			pidController.setI(i);
			kPositionI = i;	
		}
		if((d != kPositionD)) {
			pidController.setD(d);
			kPositionD = d;	
		}
		if((iz != kPositionIz)) {
			pidController.setIZone(iz);
			kPositionIz = iz;	
		}
		if((ff != kPositionFF)) {
			pidController.setFF(ff);
			kPositionFF = ff;	
		}
		if((max != kPositionMaxOutput) || (min != kPositionMinOutput)) {
			pidController.setOutputRange(min, max);
			kPositionMaxOutput = max;	
			kPositionMinOutput = min;	
		}
	
	}

	public double getHomeEncoderHeading(AnalogInput encoder){
		double heading = encoder.getValue();
		heading = ((heading+1)*360)/4096; // converting heading from tics (ranging from 0 to 4095) to degrees (ranging from 1 to 0)
		if(heading>180){
			heading = heading-360;         // converting from 1-360 degrees to -180 to 180 degrees
		}
		return -heading;
	}

	public void zeroRelativeEncoders(){
		if (rightFrontAzimuthEncoder != null) {
			rightFrontAzimuthEncoder.setPosition(0);
			leftFrontAzimuthEncoder.setPosition(0);
			leftBackAzimuthEncoder.setPosition(0);
			rightBackAzimuthEncoder.setPosition(0);
		}
	}

	public void fixRelativeEncoders(){
		if (rightFrontAzimuthEncoder != null) {
			rightFrontAzimuthEncoder.setPosition(getHomeEncoderHeading(rightFrontHomeEncoder) - RIGHT_FRONT_ABSOLUTE_OFFSET);
			leftFrontAzimuthEncoder.setPosition(getHomeEncoderHeading(leftFrontHomeEncoder) - LEFT_FRONT_ABSOLUTE_OFFSET);
			leftBackAzimuthEncoder.setPosition(getHomeEncoderHeading(leftBackHomeEncoder) - LEFT_BACK_ABSOLUTE_OFFSET);
			rightBackAzimuthEncoder.setPosition(getHomeEncoderHeading(rightBackHomeEncoder) - RIGHT_BACK_ABSOLUTE_OFFSET);
		}
	}

	public double getFixedPosition(RelativeEncoder encoder) {
		if (encoder != null) {
			double azimuth = encoder.getPosition();
			azimuth = azimuth % 360;
			if (azimuth > 180) {
				azimuth = -360 + azimuth;
			}
			if (azimuth < -180) {
				azimuth = 360 + azimuth;
			}

			// DEW 2022.01.21 not sure this should be here...
			// azimuth = Math.round(azimuth);

			return azimuth;
		} else {
			return 0;
		}
	}

	public Vector readModuleEncoders(RelativeEncoder azimuthEncoder, RelativeEncoder speedEncoder) { 
		if(azimuthEncoder != null) {
			double azimuth = azimuthEncoder.getPosition();
			if(speedEncoder != null) {
				double wheelSpeed = speedEncoder.getVelocity(); //tics per 100ms

				Vector rv = new Vector(azimuth, wheelSpeed);
				return rv;
			}
		}
		return new Vector(0,0);
	}

	public double convertDirection(double azimuthTics) { //took this out to make it testable
		double encoderReading = azimuthTics;
		double azimuth = (360.0*encoderReading)/AZIMUTH_ENCODER_CONVERSION_FACTOR;
		azimuth = azimuth % 360;
		if (azimuth > 180){
			azimuth = -360 + azimuth;
		}
		if (azimuth < -180){
			azimuth = 360 + azimuth;
		} 
		
		azimuth = Math.round(azimuth);
		
		return azimuth;
	}

	public double convertVelocity(double velocity) { //took this out to make it testable
		double encoderSpeed = velocity; //RPM
		encoderSpeed = encoderSpeed/60; //Motor Revolutions per second

		double wheelSpeed = encoderSpeed/wheelToEncoderRatioVelocity(); //wheel revolutions per second
		wheelSpeed = wheelSpeed/WHEEL_CIRCUMFERENCE; //inches per second

		return wheelSpeed;
	}

	public Vector convertSingleVector(Vector v) {
		double speed = v.getMagnitude();
		speed = speed * WHEEL_CIRCUMFERENCE;
		speed = speed * wheelToEncoderRatioVelocity();
		speed = speed * SPEED_ENCODER_TICS;
		speed = speed * 10; //converted to tics/100ms

		double direction = v.getDirection();
		if(direction < 0){
			direction = direction + 360;//converts back to [0, 360] degrees
		}
		direction = (direction*AZIMUTH_ENCODER_CONVERSION_FACTOR)/360; // converts to encoder tics 
		Vector rv = new Vector(direction, speed);

		return rv;
	}

	public DriveVectors convertAllVectors(DriveVectors dv) {
		DriveVectors rv = dv;
		rv.leftFront = convertSingleVector(dv.leftFront);
		rv.rightFront = convertSingleVector(dv.rightFront);
		rv.leftBack = convertSingleVector(dv.leftBack);
		rv.rightBack = convertSingleVector(dv.rightBack);

		return rv;
	}

	public DriveVectors getCurrentVectors() {
		DriveVectors rv = new DriveVectors();
		rv.leftFront = readModuleEncoders(leftFrontAzimuthEncoder, leftFrontDriveEncoder);
		rv.rightFront = readModuleEncoders(rightFrontAzimuthEncoder, rightFrontDriveEncoder);
		rv.leftBack = readModuleEncoders(leftBackAzimuthEncoder, leftBackDriveEncoder);
		rv.rightBack = readModuleEncoders(rightBackAzimuthEncoder, rightBackDriveEncoder);

		return rv;
	}

	public double getMaxVelocity(){
		return MAX_VELOCITY_IN_PER_SEC;
	}

	public boolean areWeStopped(){
		double totalVelocity = 0;
		if(rightFrontDriveEncoder != null){
			totalVelocity = totalVelocity + Math.abs(rightFrontDriveEncoder.getVelocity());
		}
		if(rightBackDriveEncoder != null){
			totalVelocity = totalVelocity + Math.abs(rightBackDriveEncoder.getVelocity());
		}
		if(leftFrontDriveEncoder != null){
			totalVelocity = totalVelocity + Math.abs(leftFrontDriveEncoder.getVelocity());
		}
		if(leftBackDriveEncoder != null){
			totalVelocity = totalVelocity + Math.abs(leftBackDriveEncoder.getVelocity());
		}
		
		if(totalVelocity <= 2.0){
			return true;
		} else {
			return false;
		}
	}

	public double getWheelCircumference(){
		return WHEEL_CIRCUMFERENCE;
	}

	public void switchFieldRelative(){
		fieldRelative = !fieldRelative;
	}

	public boolean getFieldRelative(){
		return fieldRelative;
	}

	public double getSpinPower(){
		return spinPower;
	}

	public void setManualSpinMode() {
        if (logSpinTransitions && autoSpinMode){
           logger.info("Switching to Manual Spin Mode");
        }
        autoSpinMode = false;
	}

	public double getTargetHeading(){
		return targetHeading;
	}

	public void setTargetHeading(double angle){
		//logger.info("setting heading to "+angle);
		targetHeading = angle;
	}

	public void setAutoSpinMode() {
        if (logSpinTransitions && !autoSpinMode){
           logger.info("Switching to Auto Spin Mode");
        }
        autoSpinMode = true;
	}

	public void setForcedManualModeTrue(){
		forceManualMode = true;
	}

	public void setForcedManualModeFalse(){
		forceManualMode = false;
	}

	public boolean getForcedManualMode(){
		return forceManualMode;
	}

	public enum Corner {
		LF, RF, LB, RB;
	}

	double getHomeOffsetForCorner(Corner corner) {
		if (swerveParameters == null) return 0;
		switch (corner) {
			case LF:
				return swerveParameters.getLeftFrontAbsoluteOffset();
			case RF:
				return swerveParameters.getRightFrontAbsoluteOffset();
			case LB:
				return swerveParameters.getLeftBackAbsoluteOffset();
			case RB:
				return swerveParameters.getRightBackAbsoluteOffset();
			default:
				return 0.0;
		}
	}

	AnalogInput getHomeEncoderForCorner(Corner corner) {
		switch (corner) {
			case LF:
				return leftFrontHomeEncoder;
			case RF:
				return rightFrontHomeEncoder;
			case LB:
				return leftBackHomeEncoder;
			case RB:
				return rightBackHomeEncoder;
			default:
				return null;
		}
	}

	RelativeEncoder getAzimuthEncoderForCorner(Corner corner) {
		switch (corner) {
			case LF:
				return leftFrontAzimuthEncoder;
			case RF:
				return rightFrontAzimuthEncoder;
			case LB:
				return leftBackAzimuthEncoder;
			case RB:
				return rightBackAzimuthEncoder;
			default:
				return null;
		}
	}

	RelativeEncoder getDriveEncoderForCorner(Corner corner) {
		switch (corner) {
			case LF:
				return leftFrontDriveEncoder;
			case RF:
				return rightFrontDriveEncoder;
			case LB:
				return leftBackDriveEncoder;
			case RB:
				return rightBackDriveEncoder;
			default:
				return null;
		}
	}

	public double homeEncoderOffset(Corner corner) {
		RelativeEncoder motorEncoder = getAzimuthEncoderForCorner(corner);
		AnalogInput homeEncoder = getHomeEncoderForCorner(corner);
		double motorEncoderHeading = (motorEncoder == null) ? 0.0 : motorEncoder.getPosition();
		return SwerveCalculator.calculateAngleDifference(motorEncoderHeading, getHomeEncoderHeading(homeEncoder));
	}

	public double encoderDifference(Corner corner) {
		RelativeEncoder motorEncoder = getAzimuthEncoderForCorner(corner);
		AnalogInput homeEncoder = getHomeEncoderForCorner(corner);
		double offset = getHomeOffsetForCorner(corner);
		double motorEncoderPosition = (motorEncoder) == null ? 0 : motorEncoder.getPosition();
		double homeEncoderPosition = getHomeEncoderHeading(homeEncoder) - offset;
		return SwerveCalculator.calculateAngleDifference(motorEncoderPosition, homeEncoderPosition);
	}

	public double getCornerAzimuthPosition(Corner corner) {
		RelativeEncoder encoder = getAzimuthEncoderForCorner(corner);
		return getFixedPosition(encoder);
	}

	public double getCornerDriveVelocity(Corner corner) {
		RelativeEncoder encoder = getDriveEncoderForCorner(corner);
		if (encoder != null) {
			return encoder.getVelocity();
		}
		return 0;
	}

	public double getCornerDrivePosition(Corner corner) {
		RelativeEncoder encoder = getDriveEncoderForCorner(corner);
		if (encoder != null) {
			return encoder.getPosition();
		}
		return 0;
	}

	private void setOneDriveClosedLoopRampRate (CANSparkMax d, double secondsToFullThrottle, String name) {
		if (d != null) {
			double was = d.getClosedLoopRampRate();
			d.setClosedLoopRampRate(secondsToFullThrottle);			
			if (name != null) {
				logger.debug ("Drive ramp rate for {} changed: {} -> }|", name, was, secondsToFullThrottle);
			}
		}
	}

	public void setDriveToRampSlowly() {
		double secondsToFullThrottle = 0.6;
		setOneDriveClosedLoopRampRate(leftFrontDrive, secondsToFullThrottle, "LF");
		setOneDriveClosedLoopRampRate(rightFrontDrive, secondsToFullThrottle, "RF");
		setOneDriveClosedLoopRampRate(leftBackDrive, secondsToFullThrottle, "LB");
		setOneDriveClosedLoopRampRate(rightBackDrive, secondsToFullThrottle, "RB");
	}

	public void setDriveToRampQuickly() {
		double secondsToFullThrottle = DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT;
		setOneDriveClosedLoopRampRate(leftFrontDrive, secondsToFullThrottle, "LF");
		setOneDriveClosedLoopRampRate(rightFrontDrive, secondsToFullThrottle, "RF");
		setOneDriveClosedLoopRampRate(leftBackDrive, secondsToFullThrottle,"LB");
		setOneDriveClosedLoopRampRate(rightBackDrive, secondsToFullThrottle, "RB");
	}

	private void setOneDriveIdle (CANSparkMax d, IdleMode idleMode) {
		if (d != null) {
			d.setIdleMode(idleMode);
		}
	}

	public void setDriveToBrake() {
		IdleMode idleMode = IdleMode.kBrake;
		setOneDriveIdle(leftFrontDrive, idleMode);
		setOneDriveIdle(rightFrontDrive, idleMode);
		setOneDriveIdle(leftBackDrive, idleMode);
		setOneDriveIdle(rightBackDrive, idleMode);
	}

	public void setDriveToCoast() {
		IdleMode idleMode = IdleMode.kCoast;
		setOneDriveIdle(leftFrontDrive, idleMode);
		setOneDriveIdle(rightFrontDrive, idleMode);
		setOneDriveIdle(leftBackDrive, idleMode);
		setOneDriveIdle(rightBackDrive, idleMode);
	}

	void setupMotorsAndEncoders() {
		rightFrontHomeEncoder = new AnalogInput(0);
		leftFrontHomeEncoder = new AnalogInput(1);
		leftBackHomeEncoder = new AnalogInput(2);
		rightBackHomeEncoder = new AnalogInput(3);
		addChild("RB home encoder", rightFrontHomeEncoder);
		addChild("LR home encoder", leftFrontHomeEncoder);
		addChild("LB home encoder", leftBackHomeEncoder);
		addChild("RB home encoder", rightBackHomeEncoder);

		CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
		boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();
		  
		// we don't *need* to use the canDeviceFinder for CAN Talons because
		// they do not put up unreasonable amounts of SPAM
		if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 1, "RF Drive") || shouldMakeAllCANDevices){
	
			rightFrontDrive = new CANSparkMaxSendable(1, MotorType.kBrushless);
			rightFrontDriveEncoder = rightFrontDrive.getEncoder();

			canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 2, "RF Azimuth");
			rightFrontAzimuth = new CANSparkMaxSendable(2, MotorType.kBrushless);
			rightFrontAzimuthEncoder = rightFrontAzimuth.getEncoder();

			canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 3, "LF Drive");
			leftFrontDrive = new CANSparkMaxSendable(3, MotorType.kBrushless);
			leftFrontDriveEncoder = leftFrontDrive.getEncoder();

			canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 4, "LF Azimuth");
			leftFrontAzimuth = new CANSparkMaxSendable(4, MotorType.kBrushless);
			leftFrontAzimuthEncoder = leftFrontAzimuth.getEncoder();

			canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 5, "LB Drive");
			leftBackDrive = new CANSparkMaxSendable(5, MotorType.kBrushless);
			leftBackDriveEncoder = leftBackDrive.getEncoder();

			canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 6, "LB Azimuth");
			leftBackAzimuth = new CANSparkMaxSendable(6, MotorType.kBrushless);
			leftBackAzimuthEncoder = leftBackAzimuth.getEncoder();

			canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 7, "RB Drive");
			rightBackDrive = new CANSparkMaxSendable(7, MotorType.kBrushless);
			rightBackDriveEncoder = rightBackDrive.getEncoder();

			canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 8, "RB Azimuth");
			rightBackAzimuth = new CANSparkMaxSendable(8, MotorType.kBrushless);
			rightBackAzimuthEncoder = rightBackAzimuth.getEncoder();

      MotorSetup driveMotorSetup = new MotorSetup().setInverted(true).setCurrentLimit(40).setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);
      MotorSetup azimuthMotorSetup = new MotorSetup().setCurrentLimit(20).setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

			driveMotorSetup.apply(rightFrontDrive);

			azimuthMotorSetup.apply(rightFrontAzimuth);

			driveMotorSetup.apply(leftFrontDrive);

			azimuthMotorSetup.apply(leftFrontAzimuth);

			driveMotorSetup.apply(leftBackDrive);

      azimuthMotorSetup.apply(leftBackAzimuth);

      driveMotorSetup.apply(rightBackDrive);

			azimuthMotorSetup.apply(rightBackAzimuth);

			addChild("RF drive motor", rightFrontDrive);
			addChild("LF drive motor", leftFrontDrive);
			addChild("LB drive motor", leftBackDrive);
			addChild("RB drive motor", rightBackDrive);
			addChild("RF azimuth motor", rightFrontAzimuth);
			addChild("LF azimuth motor", leftFrontAzimuth);
			addChild("LB azimuth motor", leftBackAzimuth);
			addChild("RB azimuth motor", rightBackAzimuth);

			rightFrontVelPID = rightFrontDrive.getPIDController();
			rightFrontPositionPID = rightFrontAzimuth.getPIDController();
			leftFrontVelPID = leftFrontDrive.getPIDController();
			leftFrontPositionPID = leftFrontAzimuth.getPIDController();
			rightBackVelPID = rightBackDrive.getPIDController();
			rightBackPositionPID = rightBackAzimuth.getPIDController();
			leftBackVelPID = leftBackDrive.getPIDController();
			leftBackPositionPID = leftBackAzimuth.getPIDController();

			rightFrontDriveEncoder.setPositionConversionFactor(getDriveEncoderConversionFactor());
			leftFrontDriveEncoder.setPositionConversionFactor(getDriveEncoderConversionFactor());
			leftBackDriveEncoder.setPositionConversionFactor(getDriveEncoderConversionFactor());
			rightBackDriveEncoder.setPositionConversionFactor(getDriveEncoderConversionFactor());

			rightFrontDriveEncoder
					.setVelocityConversionFactor((wheelToEncoderRatioVelocity() * WHEEL_CIRCUMFERENCE) / 60);
			leftFrontDriveEncoder
					.setVelocityConversionFactor((wheelToEncoderRatioVelocity() * WHEEL_CIRCUMFERENCE) / 60);
			leftBackDriveEncoder
					.setVelocityConversionFactor((wheelToEncoderRatioVelocity() * WHEEL_CIRCUMFERENCE) / 60);
			rightBackDriveEncoder
					.setVelocityConversionFactor((wheelToEncoderRatioVelocity() * WHEEL_CIRCUMFERENCE) / 60);

			rightFrontAzimuthEncoder.setPositionConversionFactor(AZIMUTH_ENCODER_CONVERSION_FACTOR);
			leftFrontAzimuthEncoder.setPositionConversionFactor(AZIMUTH_ENCODER_CONVERSION_FACTOR);
			leftBackAzimuthEncoder.setPositionConversionFactor(AZIMUTH_ENCODER_CONVERSION_FACTOR);
			rightBackAzimuthEncoder.setPositionConversionFactor(AZIMUTH_ENCODER_CONVERSION_FACTOR);

			// TODO Wat the heck???????? Why just this one???????
			// rightBackDriveEncoder.setPositionConversionFactor(1);

			setPositionPID(rightFrontPositionPID);
			setPositionPID(leftFrontPositionPID);
			setPositionPID(leftBackPositionPID);
			setPositionPID(rightBackPositionPID);

			setVelocityPID(rightFrontVelPID);
			setVelocityPID(leftFrontVelPID);
			setVelocityPID(leftBackVelPID);
			setVelocityPID(rightBackVelPID);

			rightFrontPositionPID.setFeedbackDevice(rightFrontAzimuthEncoder);
			leftFrontPositionPID.setFeedbackDevice(leftFrontAzimuthEncoder);
			leftBackPositionPID.setFeedbackDevice(leftBackAzimuthEncoder);
			rightBackPositionPID.setFeedbackDevice(rightBackAzimuthEncoder);

			rightFrontVelPID.setFeedbackDevice(rightFrontDriveEncoder);
			leftFrontVelPID.setFeedbackDevice(leftFrontDriveEncoder);
			leftBackVelPID.setFeedbackDevice(leftBackDriveEncoder);
			rightBackVelPID.setFeedbackDevice(rightBackDriveEncoder);
		}
	}

	//private final double DRIVE_ENCODER_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE*WHE--EL_TO_ENCODER_RATIO_VELOCITY;
	double getDriveEncoderConversionFactor() {
		return WHEEL_CIRCUMFERENCE*wheelToEncoderRatioVelocity();
	}
	
	//private final double WHEEL_TO_ENCODER_RATIO_VELOCITY = (1/8.31); //for every full wheel turn, the motor turns 8.31 times
	double wheelToEncoderRatioVelocity() {
		if (swerveParameters == null) return 1/8.31; // this number is as good as any in simulation
		return (1/swerveParameters.getDriveGearRatio());
	}
}