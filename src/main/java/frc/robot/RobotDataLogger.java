package frc.robot;

import java.util.EnumSet;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.logger.DataLoggerPrelude;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.Utilities;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class RobotDataLogger {
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
    if (RobotContainer.shooterSubsystem.topMotor != null) {
      dataLogger.addDataProvider("shooter.top.velocity", () -> RobotContainer.shooterSubsystem.topMotor.getVelocity());
    }
    if (RobotContainer.shooterSubsystem.bottomMotor != null) {
      dataLogger.addDataProvider("shooter.top.velocity",
          () -> RobotContainer.shooterSubsystem.bottomMotor.getVelocity());
    }

    dataLogger.addPrelude(odometryGatherer);
    // dataLogger.addDataProvider("vision.now", () -> odometryGatherer.getFPGATime());
    // dataLogger.addDataProvider("vision.fpgatimer", () -> odometryGatherer.getVisionCaptureFPGATime());
    // dataLogger.addDataProvider("vision.ts", () -> odometryGatherer.getVisionTs());
    dataLogger.addDataProvider ("vision.poseX", () -> odometryGatherer.visionPoseX());
    dataLogger.addDataProvider ("vision.poseY", () -> odometryGatherer.visionPoseY());
    dataLogger.addDataProvider ("vision.age", () -> odometryGatherer.getVisionAge());
		dataLogger.addDataProvider("odometry.poseX", () -> odometryGatherer.odometryPoseX());
		dataLogger.addDataProvider("odometry.poseY", () -> odometryGatherer.odometryPoseY());
		dataLogger.addDataProvider("odometry.heading", () -> odometryGatherer.odometryHeading());
    dataLogger.addDataProvider("vision.targetID", () -> odometryGatherer.getTargetID());
    dataLogger.addDataProvider("vision.targetTx", () -> odometryGatherer.getTargetTx());
    dataLogger.addDataProvider("vision.targetTy", () -> odometryGatherer.getTargetTy());
  }

  void addSwerveDataLoggers(DataLogger dataLogger) {
    for (var m : RobotContainer.swerveAzimuthMotors.entrySet()) {
      addMotorProviders(dataLogger, "swerve." + m.getKey() + ".azimuth", m.getValue(),
          EnumSet.allOf(MotorFields.class));

    }
    for (var m : RobotContainer.swerveDriveMotors.entrySet()) {
      addMotorProviders(dataLogger, "swerve." + m.getKey() + ".drive", m.getValue(), EnumSet.allOf(MotorFields.class));
    }
  }

  void addMotorProviders(DataLogger dataLogger, String name, Object motor) {
    addMotorProviders(dataLogger, name, motor,
        EnumSet.of(MotorFields.CURRENT, MotorFields.TEMPERATURE, MotorFields.OUTPUT));
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
    } else if (motor instanceof TalonFX) {
      TalonFX m = (TalonFX) motor;
      if (fields.contains(MotorFields.TEMPERATURE))
        dataLogger.addDataProvider(name + ".temperature", () -> DataLogger.f2(m.getDeviceTemp().getValueAsDouble()));
      if (fields.contains(MotorFields.CURRENT))
        dataLogger.addDataProvider(name + ".current", () -> DataLogger.f2(m.getStatorCurrent().getValueAsDouble()));
      if (fields.contains(MotorFields.OUTPUT))
        dataLogger.addDataProvider(name + ".output", () -> DataLogger.f2(m.getDutyCycle().getValue()));
    }
  }

  enum MotorFields {
    TEMPERATURE, CURRENT, OUTPUT
  }

  class OdometryGatherer implements DataLoggerPrelude {
    LimelightHelpers.LimelightResults limelightResults;
    LimelightTarget_Fiducial lastTargetFiducial;
    Pose2d visionPose2d, odometryPose2d;
    Rotation2d odometryHeading;
    long fpgaTime;

    @Override
    public void dataLoggerPrelude() {
      limelightResults = RobotContainer.visionSubsystem.getLastLimelightResults();
      lastTargetFiducial = RobotContainer.visionSubsystem.getLastTargetFiducial();
      visionPose2d = RobotContainer.visionSubsystem.getLastPose2d();
      fpgaTime = RobotController.getFPGATime();

      odometryPose2d = RobotContainer.drivebase.getPose();
      odometryHeading = RobotContainer.drivebase.getHeading();
    }

    public String visionPoseX() {
      if (visionPose2d == null) return "";
      return DataLogger.f2(visionPose2d.getX());
    }
    
    public String visionPoseY() {
      if (visionPose2d == null) return "";
      return DataLogger.f2(visionPose2d.getY());
    }

    public String getVisionAge() {
      if (limelightResults == null) return "";
      // convert microseconds to seconds
      return DataLogger.f2((fpgaTime - limelightResults.targetingResults.timestamp_RIOFPGA_capture) / 1000000);
    }

    public String getFPGATime() {
      return Double.toString(fpgaTime);
    }
    
    public String getVisionCaptureFPGATime() {
      if (limelightResults == null) return "";
      // convert microseconds to seconds
      return Double.toString(limelightResults.targetingResults.timestamp_RIOFPGA_capture);
    }

    public String getVisionTs() {
      if (limelightResults == null) return "";
      return DataLogger.f2(limelightResults.targetingResults.timestamp_LIMELIGHT_publish);
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