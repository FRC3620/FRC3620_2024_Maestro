// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Vector;
//import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//import frc.robot.FieldLayout;
////


public class VisionSubsystem extends SubsystemBase {

  //private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000"); one to two inches off from around 60 inches, 2-3 inches off mid field, 3-4 inches off from 3/4 fields

  //private PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
  //private PhotonCamera cam = new PhotonCamera("HD_USB_Camera");
  public PhotonCamera cam = new PhotonCamera("USB_2M_GS_camera"); //around one inch off from around 60 inches, around 2 inches off from mid, 2 inches off from around 3/4 field

  AprilTagFieldLayout fieldLayout;

  
  int PipelineIndex = 0;
  Double camHeight = 1.0;//Ft, change accordingly
  Double angCam = 30.0;//Degrees, change accordingly
 
  
   public static final double targetWidth =
            Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight =
            Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos =
            Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final double kFarTgtZPos =
            (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

    public static final Pose3d kFarTargetPose =
            new Pose3d(
                    new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));
  /** Creates a new Vision. */
  public VisionSubsystem() {
        SmartDashboard.putString("hey", "Hi, Gavin");

            
        try {
                fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
                e.printStackTrace();
                fieldLayout = null;
        }
  }

  @Override
  public void periodic() {

  cam.setPipelineIndex(PipelineIndex);
    // This method will be called once per scheduler run
    var res = cam.getLatestResult();
    if (res.hasTargets()) {

        if(PipelineIndex == 0){//apriltags

                var imageCaptureTime = res.getTimestampSeconds();
                var bestTarget = res.getBestTarget();
                SmartDashboard.putNumber("target.id", bestTarget.getFiducialId());
                SmartDashboard.putNumber("target.area", bestTarget.getArea());

                // int bestTargetID = bestTarget.getFiducialId(); //gets target ID number


                // SmartDashboard.putNumber("bestTargetID", bestTargetID);

                var camToTargetTransform = bestTarget.getBestCameraToTarget();
                SmartDashboard.putNumber("transform.x", camToTargetTransform.getX());
                SmartDashboard.putNumber("transform.y", camToTargetTransform.getY());
                SmartDashboard.putNumber("transform.z", camToTargetTransform.getZ());
                Rotation3d rotation3d = camToTargetTransform.getRotation();
                SmartDashboard.putNumber("rotation.x", rotation3d.getX());
                SmartDashboard.putNumber("rotation.y", rotation3d.getY());
                SmartDashboard.putNumber("rotation.z", rotation3d.getZ());
                //var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());

                var bestTagPose = fieldLayout.getTagPose(bestTarget.getFiducialId()); //Position of best tag on field

                if( bestTagPose.isPresent()){

                        var bestTagPoseValue = bestTagPose.get();//Getting true value of bestTagPose
                        Transform3d targetToCamTransform = camToTargetTransform.inverse();

                        var camPose = bestTagPoseValue.plus(targetToCamTransform);

                        SmartDashboard.putNumber("tagPose.X",bestTagPoseValue.getX()); //X of Tag
                        SmartDashboard.putNumber("tagPose.Y",bestTagPoseValue.getY()); //Y of Tag
                        SmartDashboard.putNumber("tagPose.Z",bestTagPoseValue.getZ()); //Z of Tag
                
                        Rotation3d rotationCam = camPose.getRotation();

                        //Future: Incorperate with gyro on robot for angle and more accurate distance

                        //printing current camera pose and rotation
                        SmartDashboard.putNumber("robotPose.X",camPose.getX());
                        SmartDashboard.putNumber("robotPose.Y",camPose.getY());
                        SmartDashboard.putNumber("robotPose.Z",camPose.getZ());

                        SmartDashboard.putNumber("rotationOfRobot.x", rotationCam.getX()); //if robot flips(andrew)
                        SmartDashboard.putNumber("rotationOfRobot.y", rotationCam.getY()); //up and down
                        SmartDashboard.putNumber("rotationOfRobot.z", rotationCam.getZ()); //left and right
             
                }
        }
        if(PipelineIndex == 1){//ringdetection

                var imageCaptureTime = res.getTimestampSeconds();
                var bestTarget = res.getBestTarget();
                SmartDashboard.putNumber("target.id", bestTarget.getFiducialId());
                SmartDashboard.putNumber("target.area", bestTarget.getArea());

                double pitch = bestTarget.getPitch(); //degrees

                double camAngToTarget = 90-(angCam-pitch); //degrees

                double camDistToCenterTarget = Math.tan(Math.toRadians(camAngToTarget));

                var confidence = bestTarget.getPoseAmbiguity();

                SmartDashboard.putNumber("DistanceToCenterTarget", camDistToCenterTarget);
                SmartDashboard.putNumber("confidence", confidence);

        }
}
}
public vectorToSpeaker camDistanceToTargetSpeaker(){//vision portion; based on numbers, rotate and drive
        var res = cam.getLatestResult();
        var bestTarget = res.getBestTarget();
                if(bestTarget.getFiducialId()==4){

                var camToTargetTransform = bestTarget.getBestCameraToTarget();

                vectorToSpeaker result = new vectorToSpeaker();
                
                result.distance = 0;
                result.yaw = 0;

                return result;
                }else{
                        return null;
                }
        
        }
public static class vectorToSpeaker{

        double distance;
        double yaw;


}

     
}   
        




