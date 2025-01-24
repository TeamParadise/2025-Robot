/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag.io;

import com.team1165.robot.subsystems.vision.apriltag.constants.ATVisionConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * {@link ATVisionIO} class that implements a AprilTag pose estimation camera powered by a
 * coprocessor running PhotonVision.
 */
public class ATVisionIOPhoton implements ATVisionIO {
  // Camera object
  protected final PhotonCamera camera;

  /**
   * Creates a new {@link ATVisionIOPhoton} with the provided name.
   *
   * @param name The configured name of the camera.
   */
  public ATVisionIOPhoton(String name) {
    camera = new PhotonCamera(name);
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

    // Save tag IDs and pose observations to add to inputs later
    Set<Short> tagIds = new HashSet<>();
    Queue<CameraPoseObservation> poseObservations = new ArrayDeque<>(5);

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
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                result.getTimestampSeconds())); // Timestamp
      } else if (!result.targets.isEmpty()) { // Check and see if there are any targets at all
        // Do a single tag calculation instead, get the tag and it's field pose
        PhotonTrackedTarget tag = result.targets.get(0);
        Optional<Pose3d> tagPose = ATVisionConstants.aprilTagLayout.getTagPose(tag.fiducialId);

        if (tagPose.isPresent()) { // If the tag exists in the AT layout, run single tag calculation
          // Calculate robot pose based on the tag pose
          Transform3d cameraToTarget = tag.bestCameraToTarget;
          Transform3d fieldToRobot =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation())
                  .plus(cameraToTarget.inverse()) // Turn into camera field transform
                  .plus(robotToCamera.inverse()); // Turn into robot field transform
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) tag.fiducialId);

          // Add pose observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  tag.poseAmbiguity, // Ambiguity
                  1, // Tag count (one because single tag)
                  cameraToTarget.getTranslation().getNorm())); // Tag distance
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(PoseObservation[]::new);

    // Save tag IDs to inputs object
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
