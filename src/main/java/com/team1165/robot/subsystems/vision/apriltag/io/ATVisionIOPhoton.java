/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag.io;

import com.team1165.robot.subsystems.vision.apriltag.constants.ATVisionConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * {@link ATVisionIO} class that implements a AprilTag pose estimation camera powered by a
 * coprocessor running PhotonVision.
 */
public class ATVisionIOPhoton implements ATVisionIO {
  // Camera and camera intrinsics objects
  protected final PhotonCamera camera;
  private Matrix<N3, N3> camIntrinsics;
  private boolean trigEnabled = false;

  /**
   * Creates a new {@link ATVisionIOPhoton} with the provided name.
   *
   * @param name The configured name of the camera.
   */
  public ATVisionIOPhoton(String name) {
    camera = new PhotonCamera(name);
    camIntrinsics = camera.getCameraMatrix().orElse(null);
  }

  /**
   * Updates a {@link ATVisionIOInputs} instance with the latest updates from this {@link
   * ATVisionIO}.
   *
   * @param inputs A {@link ATVisionIOInputs} instance to update.
   */
  @Override
  public void updateInputs(ATVisionIOInputs inputs) {
    // Save camera connection status to inputs object
    inputs.connected = camera.isConnected();

    // Make sure the name of the camera is saved to inputs
    inputs.name = camera.getName();

    // Save tag IDs and pose/single-tag observations to add to inputs later
    Set<Short> tagIds = new HashSet<>();
    Queue<CameraPoseObservation> poseObservations = new ArrayDeque<>(5);
    Queue<SingleTagObservation> singleTagObservations = new ArrayDeque<>(5);

    // Read new camera observations
    for (var result : camera.getAllUnreadResults()) {
      // Use MultiTag instead of single tag, if possible
      if (result.multitagResult.isPresent()) {
        // Get latest MultiTag result
        MultiTargetPNPResult multitagResult = result.multitagResult.get();

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var tag : result.targets) {
          totalTagDistance += tag.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add pose observation
        poseObservations.add(
            new CameraPoseObservation(
                new Pose3d(
                    multitagResult.estimatedPose.best.getTranslation(),
                    multitagResult.estimatedPose.best.getRotation()), // 3D pose estimate
                Pose3d.kZero, // MultiTag, no alternative pose
                totalTagDistance / result.targets.size(), // Average tag distance
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                result.getTimestampSeconds())); // Timestamp
      } else if (!result.targets.isEmpty()) { // Check and see if there are any tags/targets at all
        // Do a single tag calculation instead, get the tag/target
        PhotonTrackedTarget tag = result.targets.get(0);

        if (trigEnabled) { // If we are using trig, calculate tx, ty, etc.
          // Find the center of the tag and calculate corrected value based on camera intrinsics
          List<TargetCorner> tagCorners = tag.detectedCorners;
          double sumX = 0.0;
          double sumY = 0.0;
          for (TargetCorner corner : tagCorners) {
            sumX += corner.x;
            sumY += corner.y;
          }

          // If camera intrinsics do exist, continue
          if (camIntrinsics != null) {
            double fx = camIntrinsics.get(0, 0);
            double cx = camIntrinsics.get(0, 2);
            double yawOffset = cx - sumX / 4;

            double fy = camIntrinsics.get(1, 1);
            double cy = camIntrinsics.get(1, 2);
            double pitchOffset = cy - sumY / 4;

            // Calculate the yaw (horizontal angle, x value)
            var yaw = new Rotation2d(fx, yawOffset);
            // Calculate the pitch (vertical angle, y value) with the new yaw value
            var pitch = new Rotation2d(fy / Math.cos(Math.atan(yawOffset / fx)), -pitchOffset);

            // Add tag ID
            tagIds.add((short) tag.fiducialId);

            // Add single tag observation
            singleTagObservations.add(
                new SingleTagObservation(
                    yaw.getRadians(), // Yaw (horizontal angle, x value)
                    pitch.getRadians(), // Pitch (vertical angle, y value)
                    tag.fiducialId, // Tag ID
                    tag.bestCameraToTarget.getTranslation().getNorm(), // Tag distance
                    result.getTimestampSeconds())); // Timestamp
          } else {
            // Update camera matrix if it doesn't exist
            camIntrinsics = camera.getCameraMatrix().orElse(null);
          }
        } else {
          Optional<Pose3d> tagPose = ATVisionConstants.aprilTagLayout.getTagPose(tag.fiducialId);

          // Check if the tag exists in the field layout, and if so, continue with calculation
          if (tagPose.isPresent()) {
            // Calculate camera pose based on the tag pose
            Transform3d bestCameraToTarget = tag.bestCameraToTarget;
            Pose3d bestCameraPose = tagPose.get().plus(bestCameraToTarget.inverse());

            // Get alternative camera pose
            Transform3d alternateCameraToTarget = tag.altCameraToTarget;
            Pose3d alternateCameraPose = tagPose.get().plus(alternateCameraToTarget.inverse());

            // Add tag ID
            tagIds.add((short) tag.fiducialId);

            // Add pose observation
            poseObservations.add(
                new CameraPoseObservation(
                    bestCameraPose, // Best pose estimate
                    alternateCameraPose, // Alternate pose estimate
                    bestCameraToTarget.getTranslation().getNorm(), // Tag distance
                    tag.poseAmbiguity, // Ambiguity
                    1, // Tag count (one because single tag)
                    result.getTimestampSeconds())); // Timestamp
          }
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(CameraPoseObservation[]::new);

    // Save single tag pose
    inputs.singleTagObservations = singleTagObservations.toArray(SingleTagObservation[]::new);

    // Save tag IDs to inputs object
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /**
   * Method to enable or disable single-tag pose estimation through Tx/Ty measurements. By disabling
   * this, the standard 3D solver method will be used for single-tag estimation.
   *
   * @param enable Boolean that represents whether to enable or disable Tx/Ty estimation.
   */
  @Override
  public void setSingleTagTrig(boolean enable) {
    trigEnabled = enable;
  }
}
