package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

public class NavXNavigationSubsystem extends SubsystemBase implements INavigationSubsystem {
  public AHRS ahrs;

  double headingOffset = 0;

  /** Creates a new NavXNavigationSubsystem. */
  public NavXNavigationSubsystem() {
    ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
    addChild("NavX", ahrs);
    SmartDashboard.putNumber("navx.offset", 0);
  }

  @Override
  public void periodic() {
		SmartDashboard.putNumber("navx.heading_corrected", getCorrectedHeading());
		SmartDashboard.putNumber("navx.heading_raw", getRawHeading());
    SmartDashboard.putNumber("navx.pitch", ahrs.getPitch());
    SmartDashboard.putNumber("navx.roll", ahrs.getRoll());
  }

  @Override
  public void reset() {
    ahrs.reset();
  }

  @Override
  public double getRawHeading() {
    //returns raw degrees, can accumulate past 360 or -360 degrees 
    double angle = ahrs.getAngle();
    return angle;
  }

  @Override
  public void setHeadingOffset(double headingOffset) {
    this.headingOffset = headingOffset;
		SmartDashboard.putNumber("navx.offset", headingOffset);
  }

  @Override
  public double getHeadingOffset() {
    return headingOffset;
  }
  
  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }
}
