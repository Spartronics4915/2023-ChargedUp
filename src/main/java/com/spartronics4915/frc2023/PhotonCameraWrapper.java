/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package com.spartronics4915.frc2023;

 import edu.wpi.first.apriltag.AprilTag;
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.Pair;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.wpilibj.Timer;
 import com.spartronics4915.frc2023.Constants.FieldConstants;
 import com.spartronics4915.frc2023.Constants.VisionConstants;

import java.io.IOException;
import java.util.ArrayList;
 import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
 
 public class PhotonCameraWrapper {
     public PhotonCamera photonCamera;
     public PhotonPoseEstimator robotPoseEstimator;
 
     public PhotonCameraWrapper() {
         AprilTagFieldLayout atfl;
        try {
            atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            atfl = null;
        }
 
         // Forward Camera
         photonCamera =
                 new PhotonCamera(
                         VisionConstants
                                 .cameraName); // Change the name of your camera here to whatever it is in the
         // PhotonVision UI.
        //  photonCamera.setLED(VisionLEDMode.kOn);
        //  System.out.println("LED thing");
         // ... Add other cameras here
 
         // Assemble the list of cameras & mount locations
         var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
         camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.robotToCam));
 
         robotPoseEstimator =
                 new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY, photonCamera, VisionConstants.robotToCam);
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
      *     of the observation. Assumes a planar field and the robot is always firmly on the ground
      */
     public Pair<Pose2d, Double> getEstimatedGlobalPose() {
 
         double currentTime = Timer.getFPGATimestamp();
         Optional<EstimatedRobotPose> result = robotPoseEstimator.update();
         if (result.isPresent()) {
             return new Pair<Pose2d, Double>(
                     result.get().estimatedPose.toPose2d(), currentTime - result.get().timestampSeconds);
         } else {
             return new Pair<Pose2d, Double>(null, 0.0);
         }
     }
 }