package frc.robot;

import java.util.EnumSet;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.logger.DataLoggerPrelude;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class RobotDataLogger {
  Logger logger = EventLogging.getLogger(getClass());
  PowerDistribution powerDistribution = RobotContainer.powerDistribution;
  Runtime runtime = Runtime.getRuntime();
  OdometryGatherer odometryGatherer = new OdometryGatherer();

  public RobotDataLogger(DataLogger dataLogger, CANDeviceFinder canDeviceFinder) {
    dataLogger.addDataProvider("matchTime", () -> DataLogger.f2(DriverStation.getMatchTime()));
    dataLogger.addDataProvider("robotMode", () -> Robot.getCurrentRobotMode().toString());
    dataLogger.addDataProvider("robotModeInt", () -> Robot.getCurrentRobotMode().ordinal());
    dataLogger.addDataProvider("batteryVoltage", () -> DataLogger.f2(RobotController.getBatteryVoltage()));

    if (powerDistribution != null) {
      dataLogger.addDataProvider("pdp.totalCurrent", () -> DataLogger.f2(powerDistribution.getTotalCurrent()));
      dataLogger.addDataProvider("pdp.totalPower", () -> DataLogger.f2(powerDistribution.getTotalPower()));
      dataLogger.addDataProvider("pdp.totalEnergy", () -> DataLogger.f2(powerDistribution.getTotalEnergy()));
    }

    runtime = Runtime.getRuntime();

    dataLogger.addDataProvider("mem.free", () -> runtime.freeMemory());
    dataLogger.addDataProvider("mem.total", () -> runtime.totalMemory());
    dataLogger.addDataProvider("mem.max", () -> runtime.maxMemory());

    ChassisSpeeds cs = RobotContainer.drivebase.getRobotVelocity();
    var x = cs.vxMetersPerSecond;
    var y = cs.vyMetersPerSecond;
    var velocity = Math.sqrt(x*x + y*y);
    dataLogger.addDataProvider("swerve.chassis.velocity", () -> velocity);

    addSwerveDataLoggers(dataLogger);

    // weAreAiming
    dataLogger.addDataProvider("weAreAiming", () -> RobotContainer.drivebase.getAreWeAiming());
    // motor temps
    if (RobotContainer.climbElevationSubsystem.motor != null) {
      dataLogger.addDataProvider("climber.temperature",
          () -> RobotContainer.climbElevationSubsystem.motor.getMotorTemperature());
    }
    // we have a target?
    dataLogger.addDataProvider("doWeHaveATarget", () -> RobotContainer.visionSubsystem.getCamYawToSpeaker() != null);
    // requested vs actual headings
    // shooter motor velocities
    if (RobotContainer.shooterWheelsAndAmpBarSubsystem.topMotor != null) {
      dataLogger.addDataProvider("shooter.top.velocity",
        () -> RobotContainer.shooterWheelsAndAmpBarSubsystem.topMotor.getVelocity());
    }
    if (RobotContainer.shooterWheelsAndAmpBarSubsystem.bottomMotor != null) {
      dataLogger.addDataProvider("shooter.top.velocity",
        () -> RobotContainer.shooterWheelsAndAmpBarSubsystem.bottomMotor.getVelocity());
    }
    //dataLogger.addDataProvider("shooter.elevation.position.requested", RobotContainer.shooterSubsystem.getRequestedShoulderPosition());
    dataLogger.addDataProvider("shooter.elevation.position.adjustment", () -> RobotContainer.shooterElevationSubsystem.getElevationAdjustment());
    dataLogger.addDataProvider("shooter.elevation.position.actual", () -> RobotContainer.shooterElevationSubsystem.getActualElevationPosition());

    dataLogger.addPrelude(odometryGatherer);
    // dataLogger.addDataProvider("vision.now", () -> odometryGatherer.getFPGATime());
    // dataLogger.addDataProvider("vision.fpgatimer", () -> odometryGatherer.getVisionCaptureFPGATime());
    // dataLogger.addDataProvider("vision.ts", () -> odometryGatherer.getVisionTs());
    dataLogger.addDataProvider ("vision.poseX", () -> odometryGatherer.visionPoseX());
    dataLogger.addDataProvider ("vision.poseY", () -> odometryGatherer.visionPoseY());
    dataLogger.addDataProvider ("vision.heading", () -> odometryGatherer.visionPoseRotation());
    dataLogger.addDataProvider ("vision.age", () -> odometryGatherer.getVisionAge());
		dataLogger.addDataProvider("odometry.poseX", () -> odometryGatherer.odometryPoseX());
		dataLogger.addDataProvider("odometry.poseY", () -> odometryGatherer.odometryPoseY());
		dataLogger.addDataProvider("odometry.heading", () -> odometryGatherer.odometryHeading());
    dataLogger.addDataProvider("vision.targetID", () -> odometryGatherer.getTargetID());
    dataLogger.addDataProvider("vision.targetTx", () -> odometryGatherer.getTargetTx());
    dataLogger.addDataProvider("vision.targetTy", () -> odometryGatherer.getTargetTy());
    dataLogger.addDataProvider("red.poseX", () -> odometryGatherer.getRedPoseX());
    dataLogger.addDataProvider("red.poseY", () -> odometryGatherer.getRedPoseY());
    dataLogger.addDataProvider("red.poseRot", () -> odometryGatherer.getRedPoseRotation());
    dataLogger.addDataProvider("blue.poseX", () -> odometryGatherer.getBluePoseX());
    dataLogger.addDataProvider("blue.poseY", () -> odometryGatherer.getBluePoseY());
    dataLogger.addDataProvider("blue.poseRot",() -> odometryGatherer.getBluePoseRotation());
  }

  void addSwerveDataLoggers(DataLogger dataLogger) {
    for (var m : RobotContainer.swerveAzimuthMotors.entrySet()) {
      addMotorProviders(dataLogger, "swerve." + m.getKey() + ".azimuth", m.getValue().getMotor(), EnumSet.allOf(MotorFields.class));
    }
    for (var m : RobotContainer.swerveDriveMotors.entrySet()) {
      addMotorProviders(dataLogger, "swerve." + m.getKey() + ".drive", m.getValue().getMotor(), EnumSet.allOf(MotorFields.class));
    }
  }

  void addMotorProviders(DataLogger dataLogger, String name, Object motor) {
    addMotorProviders(dataLogger, name, motor,
        EnumSet.of(MotorFields.CURRENT, MotorFields.TEMPERATURE, MotorFields.OUTPUT, MotorFields.VELOCITY));
  }

  void addMotorProviders(DataLogger dataLogger, String name, Object motor, EnumSet<MotorFields> fields) {
    if (motor instanceof CANSparkBase) {
      CANSparkBase m = (CANSparkBase) motor;
      if (fields.contains(MotorFields.TEMPERATURE))
        dataLogger.addDataProvider(name + ".temperature", () -> DataLogger.f2(m.getMotorTemperature()));
      if (fields.contains(MotorFields.CURRENT))
        dataLogger.addDataProvider(name + ".current", () -> DataLogger.f2(m.getOutputCurrent()));
      if (fields.contains(MotorFields.OUTPUT))
        dataLogger.addDataProvider(name + ".output", () -> DataLogger.f2(m.getAppliedOutput()));
      if (fields.contains(MotorFields.VELOCITY))
        dataLogger.addDataProvider(name + ".velocity", () -> DataLogger.f2(m.getEncoder().getVelocity()));
    } else if (motor instanceof TalonFX) {
      TalonFX m = (TalonFX) motor;
      if (fields.contains(MotorFields.TEMPERATURE))
        dataLogger.addDataProvider(name + ".temperature", () -> DataLogger.f2(m.getDeviceTemp().getValueAsDouble()));
      if (fields.contains(MotorFields.CURRENT))
        dataLogger.addDataProvider(name + ".current", () -> DataLogger.f2(m.getStatorCurrent().getValueAsDouble()));
      if (fields.contains(MotorFields.OUTPUT))
        dataLogger.addDataProvider(name + ".output", () -> DataLogger.f2(m.getDutyCycle().getValue()));
      if (fields.contains(MotorFields.VELOCITY))
        dataLogger.addDataProvider(name + ".velocity", () -> DataLogger.f2(m.getVelocity().getValueAsDouble()));
    } else {
      logger.error("don't know what {} is: {}", name, motor.getClass());
    }
  }

  enum MotorFields {
    TEMPERATURE, CURRENT, OUTPUT, VELOCITY
  }

  class OdometryGatherer implements DataLoggerPrelude {
    LimelightHelpers.LimelightResults limelightResults;
    LimelightTarget_Fiducial lastTargetFiducial;
    Pose2d visionPose2d, odometryPose2d;
    Rotation2d odometryHeading;
    long fpgaTime;
    LimelightHelpers.PoseEstimate red;
    LimelightHelpers.PoseEstimate blue;

    @Override
    public void dataLoggerPrelude() {
      limelightResults = RobotContainer.visionSubsystem.getLastLimelightResults();
      visionPose2d = RobotContainer.visionSubsystem.getLastPose2d();
      fpgaTime = RobotController.getFPGATime();

      odometryPose2d = RobotContainer.drivebase.getPose();
      odometryHeading = RobotContainer.drivebase.getHeading();

      red = RobotContainer.visionSubsystem.lastLimelightMeasurementRED;
      blue = RobotContainer.visionSubsystem.lastLimelightMeasurementBLUE;
    }

    public String visionPoseX() {
      if (visionPose2d == null) return "";
      return DataLogger.f2(visionPose2d.getX());
    }
    
    public String visionPoseY() {
      if (visionPose2d == null) return "";
      return DataLogger.f2(visionPose2d.getY());
    }

    public String visionPoseRotation() {
      if (visionPose2d == null) return "";
      return DataLogger.f2(visionPose2d.getRotation().getDegrees());
    }

    public String getRedPoseX() {
      if (red == null) return "";
      return DataLogger.f2(red.pose.getX());
    }
    
    public String getRedPoseY() {
      if (red == null) return "";
      return DataLogger.f2(red.pose.getY());
    }

    public String getRedPoseRotation() {
      if (red == null) return "";
      return DataLogger.f2(red.pose.getRotation().getDegrees());
    }

    public String getBluePoseX() {
      if (blue == null) return "";
      return DataLogger.f2(blue.pose.getX());
    }
    
    public String getBluePoseY() {
      if (blue == null) return "";
      return DataLogger.f2(blue.pose.getY());
    }

    public String getBluePoseRotation() {
      if (blue == null) return "";
      return DataLogger.f2(blue.pose.getRotation().getDegrees());
    }

    public String getVisionAge() {
      if (limelightResults == null) return "";
      // convert microseconds to seconds
      return DataLogger.f2((fpgaTime - limelightResults.timestamp_RIOFPGA_capture) / 1000000);
    }

    public String getFPGATime() {
      return Double.toString(fpgaTime);
    }
    
    public String getVisionCaptureFPGATime() {
      if (limelightResults == null) return "";
      // convert microseconds to seconds
      return Double.toString(limelightResults.timestamp_RIOFPGA_capture);
    }

    public String getVisionTs() {
      if (limelightResults == null) return "";
      return DataLogger.f2(limelightResults.timestamp_LIMELIGHT_publish);
    }
    
    public String odometryPoseX() {
      if (odometryPose2d == null) return "";
      double v = odometryPose2d.getX();
      if (Double.isNaN(v)) return "";
      return DataLogger.f2(v);
    }
    
    public String odometryPoseY() {
      if (odometryPose2d == null) return "";
      double v = odometryPose2d.getY();
      if (Double.isNaN(v)) return "";
      return DataLogger.f2(v);
    }
    
    public String odometryHeading() {
      if (odometryHeading == null) return "";
      return DataLogger.f2(odometryHeading.getDegrees());
    }

    public String getTargetID() {
      if (lastTargetFiducial == null) return "";
      try {
        return Integer.toString((int) lastTargetFiducial.fiducialID);
      } catch (RuntimeException ex) {
        return "";
      }
    }

    public String getTargetTx() {
      if (lastTargetFiducial == null) return "";
      return DataLogger.f2(lastTargetFiducial.tx);
    }

    public String getTargetTy() {
      if (lastTargetFiducial == null) return "";
      return DataLogger.f2(lastTargetFiducial.ty);
    }

  }

}