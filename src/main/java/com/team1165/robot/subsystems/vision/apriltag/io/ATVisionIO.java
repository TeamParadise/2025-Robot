/*
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

  /**
   * Method to enable or disable single-tag pose estimation through Tx/Ty measurements. By disabling
   * this, the standard 3D solver method will be used for single-tag estimation.
   *
   * @param enable Boolean that represents whether to enable or disable Tx/Ty estimation.
   */
  default void setSingleTagTrig(boolean enable) {}

  /** Class used to store the IO values of an AprilTag vision camera. */
  @AutoLog
  class ATVisionIOInputs {
    /** Value that represents if the camera is currently connected or not. */
    public boolean connected = false;

    /** The name of the camera, if it has one. Default is "N/A". */
    public String name = "N/A";

    /** Camera pose observations gathered since the last time the input was updated. */
    public CameraPoseObservation[] poseObservations = new CameraPoseObservation[0];

    /** Tx/Ty Single tag observations gathered since the last time the input was updated. */
    public SingleTagObservation[] singleTagObservations = new SingleTagObservation[0];

    /**
     * The AprilTag IDs that have been visible at least once since the last time the input was
     * updated.
     */
    public int[] tagIds = new int[0];
  }

  /**
   * Represents the pose observation from a AprilTag camera processed on the coprocessor.
   *
   * @param bestCameraPose The best camera result, or the pose observation itself. Note, this is the
   *     pose of the camera, not the robot.
   * @param alternateCameraPose The alternative camera result, if one exists.
   * @param averageTagDistance The average distance of the AprilTags observed from the camera.
   * @param ambiguity The measure of how "ambiguous" (unsure/unclear) the pose observation is.
   * @param tagCount The number of AprilTags used to make this pose observation.
   * @param timestamp The synced timestamp of the pose observation.
   */
  record CameraPoseObservation(
      Pose3d bestCameraPose,
      Pose3d alternateCameraPose,
      double averageTagDistance,
      double ambiguity,
      int tagCount,
      double timestamp) {}

  /**
   * Represents a single tag observation from a AprilTag camera, processed alongside the IMU
   * rotation of the robot to find the robot pose.
   *
   * @param tx The horizontal (yaw) rotation of the tag from the camera.
   * @param ty The vertical (pitch) rotation of the tag from the camera.
   * @param tagId The AprilTag ID observed in this observation.
   * @param distance The 3D distance of the tag from the camera.
   * @param timestamp The synced timestamp of the pose observation.
   */
  record SingleTagObservation(double tx, double ty, int tagId, double distance, double timestamp) {}
}
