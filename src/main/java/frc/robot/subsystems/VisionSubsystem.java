// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.Vector;
//import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.commands.TurnToCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers;
//import frc.robot.FieldLayout;
////

public class VisionSubsystem extends SubsystemBase {

    // private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000"); one
    // to two inches off from around 60 inches, 2-3 inches off mid field, 3-4 inches
    // off from 3/4 fields

    // private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
    // private PhotonCamera cam = new PhotonCamera("HD_USB_Camera");

    AprilTagFieldLayout fieldLayout;

    static Optional<Alliance> color;

    static Double camHeight = 0.546;// meters, change accordingly
    static Double angCamToObject = 30.0;// Degrees, change accordingly, facing down
    static Double angCamToApriltags = 22.0;// degrees, facing up

    static Double APRILTAGCAM_FRONT_OFFSET = .30734;// change if neccessary, add to calculations
    Double NOTEDETECTCAM_Y_OFFSET = 0.0;
    Double APRILTAGCAM_X_OFFSET = 0.0;// change if neccessary
    Double NOTEDETECTCAM_X_OFFSET = 0.0;

    boolean DoIHaveSpeakerTarget = false;

    Double camYawToSpeaker;

    Double camDistToSpeakerTag;

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
        if (results.targetingResults.targets_Fiducials.length > 0) {
            SmartDashboard.putString("Vision.DoIHaveTag", "got one");
        }
        // vectorToSpeaker result = new vectorToSpeaker();
        // gets alliance color
        color = DriverStation.getAlliance();

        // if alliance is blue...
        int desiredTargetId = (color.get() == Alliance.Blue) ? 7 : 4;
        var desiredTarget = findTargetInResults(results, desiredTargetId);

        if (desiredTarget == null) {
            camYawToSpeaker = null;
            camDistToSpeakerTag = null;
        } else {
            Pose3d camResults = desiredTarget.getCameraPose_TargetSpace();
            Pose3d targetResults = desiredTarget.getTargetPose_CameraSpace();
            var camRotation = camResults.getRotation();
            camYawToSpeaker = Math.toDegrees(camRotation.getY());
            double camPitchToSpeaker = desiredTarget.ty + angCamToApriltags;

            camDistToSpeakerTag = (1.45 - camHeight)
                    / Math.tan(Units.degreesToRadians(camPitchToSpeaker));
            
            
            SmartDashboard.putNumber("Vision.tx", desiredTarget.tx);
            SmartDashboard.putNumber("Vision.ty", desiredTarget.ty);
            /*  SmartDashboard.putNumber("Vision.camRotationPitch", camPitchToSpeaker);
            SmartDashboard.putNumber("Vision.camRotationYaw", camYawToSpeaker);*/ // don't need right now
            SmartDashboard.putNumber("Vision.TargetDistanceM", camDistToSpeakerTag);
            SmartDashboard.putNumber("Vision.TargetDistanceFt", Units.metersToFeet(camDistToSpeakerTag));

        }

    }

    public Double getCamYawToSpeaker() {
        return camYawToSpeaker;
    }

    LimelightTarget_Fiducial findTargetInResults(LimelightResults limelightResults, int id) {
        for (var target : limelightResults.targetingResults.targets_Fiducials) {
            if (target.fiducialID == id) {
                return target;
            }
        }
        return null;
    }

    public Double getCamDistToSpeaker() {

        return camDistToSpeakerTag;

    }

    public boolean doIHaveShootingSolution() {
        // If we are aiming and a target is detected and the target distance is less
        // than 4.572 meters (15 feet), Status is true
        if (SwerveSubsystem.getAreWeAiming()) {
            return true;
        }
        return false;
    }
}
