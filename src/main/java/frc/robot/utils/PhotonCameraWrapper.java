// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants;

 public class PhotonCameraWrapper {
     public PhotonCamera photonCamera;
     public PhotonPoseEstimator photonPoseEstimator;
 
     public PhotonCameraWrapper(String cameraName) {
 
        AprilTagFieldLayout atfl = null;
        try {
            atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
 
        photonCamera = new PhotonCamera(cameraName);
         
        photonPoseEstimator = new PhotonPoseEstimator(
            atfl,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
            photonCamera, 
            (cameraName == Constants.Vision.kLeftCameraName) 
                ? Constants.Vision.kLeftRobotToCam 
                : Constants.Vision.kRightRobotToCam
        );
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
      *     of the observation. Assumes a planar field and the robot is always firmly on the ground
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
         return photonPoseEstimator.update();
     }
 }