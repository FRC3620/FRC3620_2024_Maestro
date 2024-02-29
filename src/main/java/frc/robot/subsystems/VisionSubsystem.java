// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Vector;
//import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//import frc.robot.FieldLayout;
////

public class VisionSubsystem extends SubsystemBase {

    // private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000"); one
    // to two inches off from around 60 inches, 2-3 inches off mid field, 3-4 inches
    // off from 3/4 fields

    // private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
    // private PhotonCamera cam = new PhotonCamera("HD_USB_Camera");

    public PhotonCamera aprilTagCam; // around one inch off from around 60 inches, around 2 inches off from mid, 2
                                     // inches off from around 3/4 field
    public PhotonCamera noteDetectCam;

    AprilTagFieldLayout fieldLayout;

    Optional<Alliance> color;

    Double camHeight = 0.489;// meters, change accordingly
    Double angCamToObject = 30.0;// Degrees, change accordingly, facing down
    Double angCamToApriltags = 20.8;// degrees, facing up

    Double APRILTAGCAM_Y_OFFSET = 0.0;// change if neccessary, add to calculations
    Double NOTEDETECTCAM_Y_OFFSET = 0.0;
    Double APRILTAGCAM_X_OFFSET = 0.0;// change if neccessary
    Double NOTEDETECTCAM_X_OFFSET = 0.0;

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

        aprilTagCam = new PhotonCamera("AprilTagCam");

        SmartDashboard.putNumber("rotation", 0);

        noteDetectCam = new PhotonCamera("USB_2M_GS_camera(1)");

        aprilTagCam.setPipelineIndex(0);

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

    }
    public boolean doIHaveTarget(){
        var res = aprilTagCam.getLatestResult();
        if(res.hasTargets()){
            return true;
        }else{
            return false;
        }
    }

    public Double camYawToSpeaker(){
        vectorToSpeaker result = new vectorToSpeaker();
        //gets alliance color
	    color = DriverStation.getAlliance();

        //takes latest results
        var res = aprilTagCam.getLatestResult();
        
        if (res.hasTargets()) {
            //if alliance is blue...
            int desiredTargetId = (color.get() == Alliance.Blue) ? 7 : 4;
            var desiredTarget = findTargetInResults(res, desiredTargetId);
            if (desiredTarget == null) return null;
            result.yaw = -desiredTarget.getYaw();
            return result.yaw;
        }
        return null;
    }

    PhotonTrackedTarget findTargetInResults (PhotonPipelineResult photonPipelineResult, int id) {
        for (var target : photonPipelineResult.targets) {
            if (target.getFiducialId() == id) {
                return target;
            }
        }
        return null;
    }

    

    public vectorToSpeaker camDistanceToTargetSpeaker() {
        var res = aprilTagCam.getLatestResult();
        if (res.hasTargets()) {

            var bestTarget = res.getBestTarget();
            if (bestTarget.getFiducialId() == 4) {

                vectorToSpeaker result = new vectorToSpeaker();

                var camToTargetTransform = bestTarget.getBestCameraToTarget();

                double camToTargetDist = (1.45 - camHeight)
                        / Math.sin(Math.toRadians(angCamToApriltags + bestTarget.getPitch()));
                double distanceXFromTarget = camToTargetTransform.getX();
                double GD = Math.sqrt((camToTargetDist * camToTargetDist) - (camHeight * camHeight));

                result.distance = camToTargetDist;
                result.yaw = bestTarget.getYaw();
                result.distanceX = distanceXFromTarget;
                result.GD = GD;

                return result;
            } else {
                return null;
            }

        } else {
            return null;
        }

    }

    public static class vectorToSpeaker {

        double distance;
        double GD;
        double yaw;
        double distanceX;
        double distanceY;

    }

    public vectorToTag camDistanceToTag() {
        var res = aprilTagCam.getLatestResult();
        if (res != null) {
            var bestTarget = res.getBestTarget();

            vectorToTag result = new vectorToTag();

            double camToTagDist = (1.45 - camHeight) / Math.sin(Math.toRadians(angCamToApriltags));

            result.distance = camToTagDist;
            result.yaw = bestTarget.getYaw();

            return result;
        } else {
            return null;
        }
    }

    public static class vectorToTag {

        double distance;
        double yaw;

    }

    public vectorToNote camYawToNote() {
        var res = noteDetectCam.getLatestResult();
        if (res != null) {
            var bestTarget = res.getBestTarget();
            SmartDashboard.putNumber("target.id", bestTarget.getFiducialId());

            double pitch = bestTarget.getPitch(); // degrees

            double camAngToTarget = 90 - (angCamToObject - pitch); // degrees relative to horizon line

            double camDistToCenterNote = Math.tan(Math.toRadians(camAngToTarget));

            var confidenceNote = bestTarget.getPoseAmbiguity();

            vectorToNote result = new vectorToNote();

            result.distance = camDistToCenterNote;
            result.yaw = bestTarget.getYaw();
            result.confidence = confidenceNote;
            // SmartDashboard.putNumber("note Distance", result.distance);
            SmartDashboard.putNumber("note yaw", result.yaw);
            // SmartDashboard.putNumber("note confidence", result.confidence);

            return result;
        } else {
            return null;
        }
    }

    public static class vectorToNote {
        double distance;
        double yaw;
        double confidence;
    }

    void aprilTagsPeriodic() {
        var res = aprilTagCam.getLatestResult();
        if (res.hasTargets()) {
            var bestTarget = res.getBestTarget();

            var bestTagPose = fieldLayout.getTagPose(bestTarget.getFiducialId());

            if (bestTagPose.isPresent()) {
                var bestTagPoseValue = bestTagPose.get();

                double camToTargetDist = (bestTagPoseValue.getZ() - camHeight)/ Math.sin(Math.toRadians(angCamToApriltags + bestTarget.getPitch()));
                double GD = Math.sqrt((camToTargetDist * camToTargetDist) - (camHeight * camHeight)) * 1.254248946;

                double rotation = SmartDashboard.getNumber("rotation", 0);
                Rotation2d robotRotation = new Rotation2d(Math.toRadians(rotation));
                Transform2d robotTransToTarget = new Transform2d(Math.cos(Math.toRadians(rotation)) * GD,
                        Math.sin(Math.toRadians(rotation)) * GD, robotRotation);

                SmartDashboard.putNumber("GroundDist", GD);
                SmartDashboard.putNumber("robot.X", bestTagPoseValue.getX() - robotTransToTarget.getX());
                SmartDashboard.putNumber("robot.Y", bestTagPoseValue.getY() - robotTransToTarget.getY());
                SmartDashboard.putNumber("robot.Rotation", rotation);

            }

        }

    }
}