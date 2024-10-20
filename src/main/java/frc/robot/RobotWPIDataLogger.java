package frc.robot;

import java.util.EnumSet;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.DataLoggerPrelude;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.WPIDataLogger;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotWPIDataLogger {
  Logger logger = EventLogging.getLogger(getClass());
  PowerDistribution powerDistribution = RobotContainer.powerDistribution;
  Runtime runtime = Runtime.getRuntime();
  OdometryGatherer odometryGatherer = new OdometryGatherer();
  WPIDataLogger dataLogger;

  public RobotWPIDataLogger(CANDeviceFinder canDeviceFinder) {
    this.dataLogger = new WPIDataLogger();
    setup(canDeviceFinder);
  }

  public void setInterval(double s) {
    dataLogger.setInterval(s);
  }

  public void start() {
    dataLogger.start();
  }

  void setup(CANDeviceFinder canDeviceFinder) {
    dataLogger.addDoubleDataProvider("matchTime", () -> DriverStation.getMatchTime());

    dataLogger.addDoubleDataProvider("timer.getFPGATimestamp", () -> Timer.getFPGATimestamp());
    dataLogger.addDoubleDataProvider("robotcontroller.getFPGATimer", () -> RobotController.getFPGATime());

    dataLogger.addStringDataProvider("robotMode", () -> Robot.getCurrentRobotMode().toString());
    dataLogger.addLongDataProvider("robotModeInt", () -> (long) Robot.getCurrentRobotMode().ordinal());

    dataLogger.addDoubleDataProvider("batteryVoltage", () -> RobotController.getBatteryVoltage());

    if (powerDistribution != null) {
      dataLogger.addDoubleDataProvider("pdp.totalCurrent", () -> powerDistribution.getTotalCurrent());
    }

    dataLogger.addBooleanDataProvider("weAreAiming", () -> SwerveSubsystem.getAreWeAiming());

    // motor temps
    if (RobotContainer.climbElevationSubsystem.motor != null) {
      dataLogger.addDoubleDataProvider("climber.temperature",
          () -> RobotContainer.climbElevationSubsystem.motor.getMotorTemperature());
    }

    // we have a target?
    dataLogger.addBooleanDataProvider("doWeHaveATarget", () -> RobotContainer.visionSubsystem.getCamYawToSpeaker() != null);

    // shooter motor velocities
    if (RobotContainer.shooterWheelsAndAmpBarSubsystem.topMotor != null) {
      dataLogger.addDoubleDataProvider("shooter.top.actual", () -> RobotContainer.shooterWheelsAndAmpBarSubsystem.topMotor.getVelocity().getValueAsDouble());
    }
    if (RobotContainer.shooterWheelsAndAmpBarSubsystem.bottomMotor != null) {
      dataLogger.addDoubleDataProvider("shooter.bottom.actual",
          () -> RobotContainer.shooterWheelsAndAmpBarSubsystem.bottomMotor.getVelocity().getValueAsDouble());
    }

    dataLogger.addDoubleDataProvider("shooter.elevation.requested", () -> RobotContainer.shooterElevationSubsystem.getRequestedShooterElevation());
    dataLogger.addDoubleDataProvider("shooter.elevation.adjustment", () -> RobotContainer.shooterElevationSubsystem.getElevationAdjustment());
    dataLogger.addDoubleDataProvider("shooter.elevation.actual", () -> RobotContainer.shooterElevationSubsystem.getActualElevationPosition());

    dataLogger.addPrelude(odometryGatherer);

    dataLogger.addPose2dDataProvider("odometry.pose", () -> odometryGatherer.getOdometryPose2d());
    dataLogger.addDoubleDataProvider("velocity", () -> odometryGatherer.getRobotVelocity());
    dataLogger.addPose3dDataProvider("shot.pose", () -> odometryGatherer.getShotPose());
    dataLogger.addPose2dDataProvider("vision.pose", () -> odometryGatherer.getVisionPose());
    dataLogger.addDoubleDataProvider("vision.age", () -> odometryGatherer.getVisionAge());

    dataLogger.addLongDataProvider("vision.targetCount", () -> odometryGatherer.getTargetCount());
    dataLogger.addPose2dDataProvider("red.pose", () -> odometryGatherer.getRedPose());
    dataLogger.addPose2dDataProvider("blue.pose", () -> odometryGatherer.getBluePose());

    for (var m : RobotContainer.swerveAzimuthMotors.entrySet()) {
      addMotorProviders("swerve." + m.getKey() + ".azimuth", m.getValue().getMotor(),
          EnumSet.allOf(MotorFields.class));

    }
    for (var m : RobotContainer.swerveDriveMotors.entrySet()) {
      addMotorProviders("swerve." + m.getKey() + ".drive", m.getValue().getMotor(), EnumSet.allOf(MotorFields.class));
    }
  }

  void addMotorProviders(String name, Object motor) {
    addMotorProviders(name, motor,
        EnumSet.of(MotorFields.CURRENT, MotorFields.TEMPERATURE, MotorFields.OUTPUT, MotorFields.VELOCITY));
  }

  void addMotorProviders(String name, Object motor, EnumSet<MotorFields> fields) {
    if (motor instanceof CANSparkBase) {
      CANSparkBase m = (CANSparkBase) motor;
      if (fields.contains(MotorFields.TEMPERATURE))
        dataLogger.addDoubleDataProvider(name + ".temperature", () -> m.getMotorTemperature());
      if (fields.contains(MotorFields.CURRENT))
        dataLogger.addDoubleDataProvider(name + ".current", () -> m.getOutputCurrent());
      if (fields.contains(MotorFields.OUTPUT))
        dataLogger.addDoubleDataProvider(name + ".output", () -> m.getAppliedOutput());
      if (fields.contains(MotorFields.VELOCITY))
        dataLogger.addDoubleDataProvider(name + ".velocity", () -> m.getEncoder().getVelocity());
    } else if (motor instanceof TalonFX) {
      TalonFX m = (TalonFX) motor;
      if (fields.contains(MotorFields.TEMPERATURE))
        dataLogger.addDoubleDataProvider(name + ".temperature", () -> m.getDeviceTemp().getValueAsDouble());
      if (fields.contains(MotorFields.CURRENT))
        dataLogger.addDoubleDataProvider(name + ".current", () -> m.getStatorCurrent().getValueAsDouble());
      if (fields.contains(MotorFields.OUTPUT))
        dataLogger.addDoubleDataProvider(name + ".output", () -> m.getDutyCycle().getValue());
      if (fields.contains(MotorFields.VELOCITY))
        dataLogger.addDoubleDataProvider(name + ".velocity", () -> m.getVelocity().getValueAsDouble());
    } else {
      logger.error("don't know what {} is: {}", name, motor.getClass());
    }
  }

  enum MotorFields {
    TEMPERATURE, CURRENT, OUTPUT, VELOCITY
  }

  Pose2d shotPose;
  public void setTookAShot(Pose2d _shotPose) {
    shotPose = _shotPose;
  }

  class OdometryGatherer implements DataLoggerPrelude {
    private LimelightHelpers.LimelightResults limelightResults;
    private Double distanceToSpeaker, yawToSpeaker;
    private LimelightHelpers.PoseEstimate red;
    private LimelightHelpers.PoseEstimate blue;
    private Pose2d odometryPose2d, og_shotPose2d;
    private long fpgaTime;
    private ChassisSpeeds robotVelocity;

    @Override
    public void dataLoggerPrelude() {
      limelightResults = RobotContainer.visionSubsystem.getLastLimelightResults();
      fpgaTime = RobotController.getFPGATime();

      distanceToSpeaker = RobotContainer.visionSubsystem.getCamDistToSpeaker();
      yawToSpeaker = RobotContainer.visionSubsystem.getCamYawToSpeaker();

      odometryPose2d = RobotContainer.drivebase.getPose();

      red = RobotContainer.visionSubsystem.lastLimelightMeasurementRED;
      blue = RobotContainer.visionSubsystem.lastLimelightMeasurementBLUE;

      og_shotPose2d = shotPose;
      shotPose = null;

      robotVelocity = RobotContainer.drivebase.getRobotVelocity();
    }

    static final Transform3d plus1MZ = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d());
    public Pose3d getShotPose() {
      if (og_shotPose2d != null) {
        return new Pose3d(og_shotPose2d).plus(plus1MZ);
      } else {
        return new Pose3d(odometryPose2d).plus(plus1MZ);
      }
    }

    public int getTargetCount() {
      if (limelightResults == null) return 0;
      return limelightResults.targets_Fiducials.length;
    }

    public double getRobotVelocity() {
      if (robotVelocity == null) return 0;
      double x = robotVelocity.vxMetersPerSecond;
      double y = robotVelocity.vyMetersPerSecond;
      return Math.sqrt(x*x + y*y);
    }

    public Pose2d getVisionPose() {
      Pose2d rv = null;
      if (limelightResults != null) {
        rv = limelightResults.getBotPose2d_wpiBlue();
      }
      return pp(rv);
    }

    public Pose2d getVisionPose2() {
      Pose2d rv = null;
      if (limelightResults != null) {
        rv = limelightResults.getBotPose2d_wpiBlue();
      }
      return pp(rv);
    }

    public Pose2d getOdometryPose2d() {
      return pp(odometryPose2d);
    }

    public Pose2d getRedPose() {
      return (red == null) ? mars: pp(red.pose);
    }

    public Pose2d getBluePose() {
      return (blue == null) ? mars : pp(blue.pose);
    }

    public double getVisionAge() {
      if (limelightResults == null) return Double.NaN;
      // convert microseconds to seconds
      return (fpgaTime - limelightResults.timestamp_RIOFPGA_capture) / 1000000;
    }

    public long getFPGATime() {
      return fpgaTime;
    }

    public double getCamDistToSpeaker() {
      return dd(distanceToSpeaker);
    }

    public double getCamYawToSpeaker() {
      return dd(yawToSpeaker);
    }

    public double getVisionCaptureFPGATime() {
      if (limelightResults == null) return Double.NaN;
      // convert microseconds to seconds
      return limelightResults.timestamp_RIOFPGA_capture;
    }

    public double getVisionTs() {
      if (limelightResults == null) return Double.NaN;
      return limelightResults.timestamp_LIMELIGHT_publish;
    }
    
    public Pose2d odometryPose() {
      return pp(odometryPose2d);
    }

  }

  final static Pose2d mars = new Pose2d (-1000, -1000, Rotation2d.fromDegrees(0));

  double dd (Double d) {
    return (d == null) ? Double.NaN : d;
  }

  Pose2d pp (Pose2d p) {
    return (p == null) ? mars : p;
  }

}
