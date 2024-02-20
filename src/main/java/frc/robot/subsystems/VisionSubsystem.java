// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

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

    Double camHeight = 0.803275;// meters, change accordingly
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

        SmartDashboard.putNumber("rotation",0);

        noteDetectCam = new PhotonCamera("USB_2M_GS_camera(1)");

        aprilTagCam.setPipelineIndex(0);

        //noteDetectCam.setPipelineIndex(0);

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
        var res = aprilTagCam.getLatestResult();
        if (res.hasTargets()) {
            var bestTarget = res.getBestTarget();
            
            var bestTagPose = fieldLayout.getTagPose(bestTarget.getFiducialId());

            if (bestTagPose.isPresent()) {
            var bestTagPoseValue = bestTagPose.get();

            double camToTargetDist = (bestTagPoseValue.getZ() - camHeight) / Math.sin(Math.toRadians(angCamToApriltags + bestTarget.getPitch()));
            double GD = Math.sqrt((camToTargetDist*camToTargetDist) - (camHeight*camHeight))*1.254248946;

            double rotation = SmartDashboard.getNumber("rotation", 0);
            Rotation2d robotRotation = new Rotation2d(Math.toRadians(rotation));
            Transform2d robotTransToTarget = new Transform2d(Math.cos(Math.toRadians(rotation))*GD,Math.sin(Math.toRadians(rotation))*GD,robotRotation); 
            
            SmartDashboard.putNumber("GroundDist", GD);
            SmartDashboard.putNumber("robot.X", bestTagPoseValue.getX()-robotTransToTarget.getX());
            SmartDashboard.putNumber("robot.Y", bestTagPoseValue.getY()-robotTransToTarget.getY());
            SmartDashboard.putNumber("robot.Rotation", rotation);
            
            
            
            }
        }
        /*var res = aprilTagCam.getLatestResult();
        var bestTarget = res.getBestTarget();
        vectorToSpeaker camDistanceToSpeaker = camDistanceToTargetSpeaker();
        if (camDistanceToSpeaker != null) {

            double speakerYaw = camDistanceToSpeaker.yaw;
            
            double GroundDist = camDistanceToSpeaker.GD;

            SmartDashboard.putNumber("Speaker Ground Distance", GroundDist*1.254248946);
            SmartDashboard.putNumber("Speaker yaw", speakerYaw);

            double rotation = SmartDashboard.getNumber("rotation", 0);
            Rotation2d robotRotation = new Rotation2d(Math.toRadians(rotation));
            Transform2d robotTransToTarget = new Transform2d(Math.cos(rotation)*GroundDist,Math.sin(rotation)*GroundDist,robotRotation); 
            

            SmartDashboard.putNumber("Robot.X")

        }
*/
    }
    /*]
     * vectorToNote camDistanceToNote = new vectorToNote();
     * double noteConfidence = camDistanceToNote.confidence;
     * double noteYaw = camDistanceToNote.yaw;
     * double noteDistance = camDistanceToNote.distance;
     * SmartDashboard.putNumber("note Distance", noteDistance);
     * SmartDashboard.putNumber("note yaw", noteYaw);
     * SmartDashboard.putNumber("note confidence", noteConfidence);
     */

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
                double GD = Math.sqrt((camToTargetDist*camToTargetDist) - (camHeight*camHeight));

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

    public vectorToNote camDistanceToNote() {
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
            SmartDashboard.putNumber("note Distance", result.distance);
            SmartDashboard.putNumber("note yaw", result.yaw);
            SmartDashboard.putNumber("note confidence", result.confidence);

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
            var imageCaptureTime = res.getTimestampSeconds();
            var bestTarget = res.getBestTarget();
            SmartDashboard.putNumber("target.id", bestTarget.getFiducialId());

            var camToTargetTransform = bestTarget.getBestCameraToTarget();
            SmartDashboard.putNumber("transform.x", camToTargetTransform.getX());
            SmartDashboard.putNumber("transform.y", camToTargetTransform.getY());
            SmartDashboard.putNumber("transform.z", camToTargetTransform.getZ());
            Rotation3d rotation3d = camToTargetTransform.getRotation();
            SmartDashboard.putNumber("rotation.x", rotation3d.getX());
            SmartDashboard.putNumber("rotation.y", rotation3d.getY());
            SmartDashboard.putNumber("rotation.z", rotation3d.getZ());
            // var camPose =
            // Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());

            var bestTagPose = fieldLayout.getTagPose(bestTarget.getFiducialId()); // Position of best tag on
                                                                                  // field

            if (bestTagPose.isPresent()) {

                var bestTagPoseValue = bestTagPose.get();// Getting true value of bestTagPose
                Transform3d targetToCamTransform = camToTargetTransform.inverse();

                var camPose = bestTagPoseValue.plus(targetToCamTransform);

                SmartDashboard.putNumber("tagPose.X", bestTagPoseValue.getX()); // X of Tag
                SmartDashboard.putNumber("tagPose.Y", bestTagPoseValue.getY()); // Y of Tag
                SmartDashboard.putNumber("tagPose.Z", bestTagPoseValue.getZ()); // Z of Tag

                Rotation3d rotationCam = camPose.getRotation();

                // Future: Incorperate with gyro on robot for angle and more accurate distance

                // printing current camera pose and rotation
                SmartDashboard.putNumber("robotPose.X", camPose.getX());
                SmartDashboard.putNumber("robotPose.Y", camPose.getY());
                SmartDashboard.putNumber("robotPose.Z", camPose.getZ());

                SmartDashboard.putNumber("rotationOfRobot.x", rotationCam.getX()); // if robot
                                                                                   // flips(andrew)
                SmartDashboard.putNumber("rotationOfRobot.y", rotationCam.getY()); // up and down
                SmartDashboard.putNumber("rotationOfRobot.z", rotationCam.getZ()); // left and right

            }

        }

    }

}