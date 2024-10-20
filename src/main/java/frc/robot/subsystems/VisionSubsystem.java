// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Date;
import java.util.Optional;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.Utilities;

import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.RobotContainer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//import frc.robot.FieldLayout;
////

public class VisionSubsystem extends SubsystemBase {
    Logger logger = EventLogging.getLogger(getClass());

    // private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000"); one
    // to two inches off from around 60 inches, 2-3 inches off mid field, 3-4 inches
    // off from 3/4 fields

    // private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
    // private PhotonCamera cam = new PhotonCamera("HD_USB_Camera");

    AprilTagFieldLayout fieldLayout;

    static Optional<Alliance> color;

    static Double camHeight = 0.6604;// meters, change accordingly
    static Double angCamToObject = 30.0;// Degrees, change accordingly, facing down
    static Double angCamToApriltags = 22.0;// degrees, facing up

    static Double APRILTAGCAM_FRONT_OFFSET = 0.3048;// change if neccessary, add to calculations
    Double APRILTAGCAM_X_OFFSET = 0.0;// change if neccessary
    Double NOTEDETECTCAM_X_OFFSET = 0.0;
    static Double SHOOTER_AIM_OFFSET = 0.0;

    boolean DoIHaveSpeakerTarget = false;

    Double camYawToSpeaker = null;

    Double camDistToSpeakerTag = null;

    LimelightHelpers.LimelightResults lastLimelightResults = null;
    Pose2d lastPose = null;
    // LimelightTarget_Fiducial lastTargetFiducial = null;
    Double lastFPGATime = null;
    public LimelightHelpers.PoseEstimate lastLimelightMeasurementBLUE;
    public LimelightHelpers.PoseEstimate lastLimelightMeasurementRED;

    Double lastTimestamp = null;

    // Set Target Speaker Positions
    public static Translation2d blueSpeakerPos = new Translation2d(0.076, 5.547868);
    public static Translation2d redSpeakerPos = new Translation2d(16.465042, 5.547868);

    public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75)
            - Units.inchesToMeters(48.0 / 2.0);
    public static final double kFarTgtZPos = (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

    public static final Pose3d kFarTargetPose = new Pose3d(
            new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

    /** Creates a new Vision. */
    public VisionSubsystem() {

        SmartDashboard.putNumber("rotation", 0);

        // noteDetectCam.setPipelineIndex(0);

        try {
            fieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            fieldLayout = null;
        }
    }

    @Override
    public void periodic() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("");
        lastLimelightResults = results;

        if (lastTimestamp == null || lastTimestamp != results.timestamp_LIMELIGHT_publish) {
            lastTimestamp = results.timestamp_LIMELIGHT_publish;
            // fill in what time we received this
            lastFPGATime = results.timestamp_RIOFPGA_capture = RobotController.getFPGATime();
        } else {
            // has the same timestamp as the last one we saw, so use the same FPGA time
            results.timestamp_RIOFPGA_capture = lastFPGATime;
        }

        // Maybe can skip this if it's not a new sample?

        if (results.targets_Fiducials.length > 0) {
            SmartDashboard.putString("Vision.DoIHaveTag", "got one");

        }
        // vectorToSpeaker result = new vectorToSpeaker();
        // gets alliance color
        color = DriverStation.getAlliance();

        // added this to handle case where color is not yet set, otherwise we blow up in the simulator
        if (color.isEmpty()) return;

        Pose2d currentPose = lastLimelightResults.getBotPose2d_wpiBlue();

        /* 20240336 - I don't think we need these anymore. Commenting them out for now.
        // if alliance is blue.
        int desiredTargetId = (color.get() == Alliance.Blue) ? 7 : 4;
        var desiredTarget = findTargetInResults(results, desiredTargetId);
        lastTargetFiducial = desiredTarget;
         

        if (desiredTarget == null) {
            //lastPose = null;
            camYawToSpeaker = null;
            camDistToSpeakerTag = null;
        } else {*/

            if (color.get() == Alliance.Blue) {
                camDistToSpeakerTag = currentPose.getTranslation().getDistance(blueSpeakerPos)-APRILTAGCAM_FRONT_OFFSET;
                camYawToSpeaker = Utilities.normalizeAngle(currentPose.getTranslation().minus(blueSpeakerPos).getAngle().getDegrees() + SHOOTER_AIM_OFFSET);
            } else {
                camDistToSpeakerTag = currentPose.getTranslation().getDistance(redSpeakerPos)-APRILTAGCAM_FRONT_OFFSET;
                camYawToSpeaker = Utilities.normalizeAngle(currentPose.getTranslation().minus(redSpeakerPos).getAngle().getDegrees()+SHOOTER_AIM_OFFSET);

                /* Not sure if we'll need this so commenting it out for now.
                if (Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
                    camYawToSpeaker = Utilities.normalizeAngle(currentPose.getTranslation().minus(redSpeakerPos).getAngle().getDegrees()+3);
                }  else {
                    camYawToSpeaker = Utilities.normalizeAngle(currentPose.getTranslation().minus(redSpeakerPos).getAngle().getDegrees()-180+3);
                }
                */
            }

            SmartDashboard.putNumber("Vision.DistToSpeakerTag", Units.metersToFeet(camDistToSpeakerTag));
            SmartDashboard.putNumber("Vision.camYawToSpeaker", camYawToSpeaker);
        /* } */
    }

    public Double getCamYawToSpeaker() {

        // NOTE: This method returns the heading angle which points to the robot toward the target. This is FIELD-RELATIVE
        return camYawToSpeaker;
    }

    public static LimelightTarget_Fiducial findTargetInResults(LimelightResults limelightResults, int id) {
        for (var target : limelightResults.targets_Fiducials) {
            if (target.fiducialID == id) {
                return target;
            }
        }
        return null;
    }

    public Double getCamDistToSpeaker() {
        return camDistToSpeakerTag;
    }

    public LimelightHelpers.LimelightResults getLastLimelightResults() {
        return lastLimelightResults;
    }

    /*
    public LimelightTarget_Fiducial getLastTargetFiducial() {
        return lastTargetFiducial;
    }
    */

    public Pose2d getLastPose2d() {
        return lastPose;
    }

    public boolean doIHaveShootingSolution() {
        // If we are aiming and a target is detected and the target distance is less
        // than 4.572 meters (15 feet), Status is true
        if (SwerveSubsystem.getAreWeAiming()) {
            return true;
        }
        return false;
    }

    ObjectMapper objectMapper = new ObjectMapper().setVisibility(PropertyAccessor.FIELD, Visibility.PUBLIC_ONLY);

    public void takeSnapshot() {
        try {
            SnapshotData s = new SnapshotData();
            s.timestamp = LoggingMaster.convertTimestampToString(new Date());

            LimelightHelpers.takeSnapshot("", s.timestamp);

            // var lastTargetFiducial = getLastTargetFiducial();
            s.visionPose = getLastPose2d();
            s.odometryPose = RobotContainer.drivebase.getPose();

            Robot.robotWPIDataLogger.setTookAShot(s.odometryPose);

            s.actualShooterPosition = RobotContainer.shooterElevationSubsystem.getActualElevationPosition();
            s.requestedShooterPosition = RobotContainer.shooterElevationSubsystem.getRequestedShooterElevation();

            String ss = objectMapper.writeValueAsString(s);
            logger.info ("Shooting: {}", ss);
        } catch (Exception ex) {
            logger.error ("takeSnapshot() didn't work: {}", ex.getMessage());
        }
    }

    public static class SnapshotData {
        public String timestamp;
        public Pose2d odometryPose;
        public Pose2d visionPose;
        public Double requestedShooterPosition, actualShooterPosition;

    }
}