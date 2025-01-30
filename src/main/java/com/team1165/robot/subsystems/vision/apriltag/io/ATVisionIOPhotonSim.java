/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag.io;

import static com.team1165.robot.subsystems.vision.apriltag.constants.ATVisionConstants.aprilTagLayout;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * {@link ATVisionIO} class that implements a AprilTag pose estimation camera powered by a simulated
 * PhotonVision environment.
 */
public class ATVisionIOPhotonSim extends ATVisionIOPhoton {
  // Vision simulation environment with all targets and cameras
  private static final VisionSystemSim visionSim = new VisionSystemSim("apriltag");

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new {@link ATVisionIOPhotonSim} with the provided values.
   *
   * @param name The name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param cameraProperties The unique properties of the simulated camera (error, FOV, and more).
   * @param poseSupplier The supplier of the robot pose to base the camera position on.
   */
  public ATVisionIOPhotonSim(
      String name,
      Transform3d robotToCamera,
      SimCameraProperties cameraProperties,
      Supplier<Pose2d> poseSupplier) {
    // Call constructor from ATVisionIOPhoton
    super(name);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim.getVisionTargets().isEmpty()) {
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  /**
   * Creates a new {@link ATVisionIOPhotonSim} with the provided configurations.
   *
   * @param name The name of the camera.
   * @param simConfig The {@link ATVisionIOPhotonSimConfig} with the simulation configuration for
   *     this camera.
   * @param poseSupplier The supplier of the robot pose to base the camera position on.
   */
  public ATVisionIOPhotonSim(
      String name, ATVisionIOPhotonSimConfig simConfig, Supplier<Pose2d> poseSupplier) {
    // Call constructor from ATVisionIOPhoton
    super(name);
    var cameraProperties = new SimCameraProperties();

    // Set calibration based on provided values
    if (simConfig.camIntrinsics() != null && simConfig.distCoeffs() != null) {
      cameraProperties.setCalibration(
          simConfig.width(), simConfig.height(), simConfig.camIntrinsics(), simConfig.distCoeffs());
    } else if (simConfig.cameraFOV() != null) {
      cameraProperties.setCalibration(simConfig.width(), simConfig.height(), simConfig.cameraFOV());
    } else {
      cameraProperties.setCalibration(
          simConfig.width(), simConfig.height(), Rotation2d.fromDegrees(90));
    }

    // Set other calibration values
    cameraProperties.setCalibError(simConfig.avgErrorPx(), simConfig.errorStdDevPx());
    if (simConfig.fps() != 0) {
      cameraProperties.setFPS(simConfig.fps());
    }
    cameraProperties.setAvgLatencyMs(simConfig.avgLatencyMs());
    cameraProperties.setLatencyStdDevMs(simConfig.latencyStdDevMs());

    // Continue normal creation
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim.getVisionTargets().isEmpty()) {
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, simConfig.robotToCamera());
  }

  /**
   * Creates a new ATVisionIOPhotonSim. This constructor creates a 960x720 "perfect camera", with no
   * latency or error.
   *
   * @param name The name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param poseSupplier The supplier of the robot pose to base the camera position on.
   */
  public ATVisionIOPhotonSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this(name, robotToCamera, new SimCameraProperties(), poseSupplier);
  }

  /**
   * Updates a {@link ATVisionIOInputs} instance with the latest updates from this {@link
   * ATVisionIO}.
   *
   * @param inputs A {@link ATVisionIOInputs} instance to update.
   */
  @Override
  public void updateInputs(ATVisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }

  /**
   * A configuration class used to provide values needed to create an {@link ATVisionIOPhotonSim}.
   *
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param width The resolution width of the camera.
   * @param height The resolution height of the camera.
   * @param cameraFOV The diagonal FOV (field of view) of the camera. This will be ignored if {@code
   *     camIntrinsics} and {@code distCoeffs} are set.
   * @param camIntrinsics The intrinsics of the camera (can be found through PhotonVision
   *     calibration). In the format of (fx, 0, cx, 0, fy, cy, 0, 0, 1).
   * @param distCoeffs The distortion of the camera (can be found through PhotonVision calibration).
   * @param avgErrorPx The average error of the camera (can be found through PhotonVision
   *     calibration).
   * @param errorStdDevPx The standard deviation of the error of the camera.
   * @param fps The average frames per second that the camera processes at.
   * @param avgLatencyMs The average latency in milliseconds between the camera and the coprocessor.
   * @param latencyStdDevMs The standard deviation of the latency of the camera.
   */
  public record ATVisionIOPhotonSimConfig(
      int width,
      int height,
      Transform3d robotToCamera,
      Rotation2d cameraFOV,
      Matrix<N3, N3> camIntrinsics,
      Matrix<N8, N1> distCoeffs,
      double avgErrorPx,
      double errorStdDevPx,
      double fps,
      double avgLatencyMs,
      double latencyStdDevMs) {
    /**
     * Creates a new {@link ATVisionIOPhotonSimConfig} with the provided camera offset. This
     * constructor creates a 960x720 "perfect camera", with no latency or error.
     *
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public ATVisionIOPhotonSimConfig(Transform3d robotToCamera) {
      this(960, 720, robotToCamera, null, null, null, 0, 0, 0, 0, 0);
    }

    /**
     * Creates a new {@link ATVisionIOPhotonSimConfig} with the provided values. This constructor
     * assumes a "perfect camera", with no latency or error.
     *
     * @param width The resolution width of the camera.
     * @param height The resolution height of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public ATVisionIOPhotonSimConfig(int width, int height, Transform3d robotToCamera) {
      this(width, height, robotToCamera, null, null, null, 0, 0, 0, 0, 0);
    }

    /**
     * Creates a new {@link ATVisionIOPhotonSimConfig} with the provided values.
     *
     * @param width The resolution width of the camera.
     * @param height The resolution height of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     * @param cameraFOV The diagonal FOV (field of view) of the camera.
     */
    public ATVisionIOPhotonSimConfig(
        int width, int height, Transform3d robotToCamera, Rotation2d cameraFOV) {
      this(width, height, robotToCamera, cameraFOV, null, null, 0, 0, 0, 0, 0);
    }

    /**
     * Creates a new {@link ATVisionIOPhotonSimConfig} with the provided values.
     *
     * @param width The resolution width of the camera.
     * @param height The resolution height of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     * @param camIntrinsics The intrinsics of the camera (can be found through PhotonVision
     *     calibration). In the format of (fx, 0, cx, 0, fy, cy, 0, 0, 1).
     * @param distCoeffs The distortion of the camera (can be found through PhotonVision
     *     calibration).
     */
    public ATVisionIOPhotonSimConfig(
        int width,
        int height,
        Transform3d robotToCamera,
        Matrix<N3, N3> camIntrinsics,
        Matrix<N8, N1> distCoeffs) {
      this(width, height, robotToCamera, null, camIntrinsics, distCoeffs, 0, 0, 0, 0, 0);
    }

    /**
     * Creates a copy of this {@link ATVisionIOPhotonSimConfig}, with new calibration error values.
     *
     * @param avgErrorPx The average error of the camera (can be found through PhotonVision
     *     calibration).
     * @param errorStdDevPx The standard deviation of the error of the camera.
     * @return A copy of this {@link ATVisionIOPhotonSimConfig} with modified values.
     */
    public ATVisionIOPhotonSimConfig withCalibError(double avgErrorPx, double errorStdDevPx) {
      return new ATVisionIOPhotonSimConfig(
          width,
          height,
          robotToCamera,
          cameraFOV,
          camIntrinsics,
          distCoeffs,
          avgErrorPx,
          errorStdDevPx,
          fps,
          avgLatencyMs,
          latencyStdDevMs);
    }

    /**
     * Creates a copy of this {@link ATVisionIOPhotonSimConfig}, with a new FPS value.
     *
     * @param fps The average frames per second that the camera processes at.
     * @return A copy of this {@link ATVisionIOPhotonSimConfig} with modified values.
     */
    public ATVisionIOPhotonSimConfig withFPS(double fps) {
      return new ATVisionIOPhotonSimConfig(
          width,
          height,
          robotToCamera,
          cameraFOV,
          camIntrinsics,
          distCoeffs,
          avgErrorPx,
          errorStdDevPx,
          fps,
          avgLatencyMs,
          latencyStdDevMs);
    }

    /**
     * Creates a copy of this {@link ATVisionIOPhotonSimConfig}, with new latency values.
     *
     * @param avgLatencyMs The average latency in milliseconds between the camera and the
     *     coprocessor.
     * @param latencyStdDevMs The standard deviation of the latency of the camera.
     * @return A copy of this {@link ATVisionIOPhotonSimConfig} with modified values.
     */
    public ATVisionIOPhotonSimConfig withLatency(double avgLatencyMs, double latencyStdDevMs) {
      return new ATVisionIOPhotonSimConfig(
          width,
          height,
          robotToCamera,
          cameraFOV,
          camIntrinsics,
          distCoeffs,
          avgErrorPx,
          errorStdDevPx,
          fps,
          avgLatencyMs,
          latencyStdDevMs);
    }
  }
}
