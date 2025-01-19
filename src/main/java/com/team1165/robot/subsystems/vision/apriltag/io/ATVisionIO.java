/*
 * File originally made by: Mechanical Advantage - FRC 6328
 * Copyright (c) 2025 Team 6328 (https://github.com/Mechanical-Advantage)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag.io;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * A hardware interface/implementation layer for a camera that estimates the pose of the robot with
 * AprilTags placed around the field.
 */
public interface ATVisionIO {
  /**
   * Updates a {@link ATVisionIOInputs} instance with the latest updates from this {@link
   * ATVisionIO}.
   *
   * @param inputs A {@link ATVisionIOInputs} instance to update.
   */
  default void updateInputs(ATVisionIOInputs inputs) {}

  /** Class used to store the IO values of an AprilTag vision camera. */
  @AutoLog
  class ATVisionIOInputs {
    /** Value that represents if the camera is currently connected or not. */
    public boolean connected = false;

    /** Pose observations gathered since the last time the input was updated. */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    /**
     * The AprilTag IDs that have been visible at least once since the last time the input was
     * updated.
     */
    public int[] tagIds = new int[0];
  }

  /**
   * Represents a robot pose sample provided by a camera used for pose estimation.
   *
   * @param timestamp The synced timestamp of the pose observation.
   * @param pose The pose observation itself.
   * @param ambiguity The measure of how "ambiguous" (unsure/unclear) the pose observation is.
   * @param tagCount The number of AprilTags used to make this pose observation.
   * @param averageTagDistance The average distance of the AprilTags observed from the camera.
   */
  record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}
}
