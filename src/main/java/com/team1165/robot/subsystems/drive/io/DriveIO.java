/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.io;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A "software" interface/implementation layer for a Swerve Drivetrain powered by the CTRE Swerve
 * Library. Due to the nature of the CTRE Swerve Library, this IO class is limited in replay
 * capabilities. This IO layer is used to communicate with the CTRE Swerve Library in a partially
 * AdvantageKit compatible way. Any new features or customization that can be done outside of this
 * communication layer should be done in the actual Drive subsystem.
 */
public interface DriveIO {
  /**
   * Updates a {@link DriveIOInputs} instance with the latest updates from this {@link DriveIO}.
   *
   * @param inputs A {@link DriveIOInputs} instance to update.
   */
  default void updateInputs(DriveIOInputs inputs) {}

  /**
   * Logs specific additional information about the swerve modules, like temperature, applied
   * output, current, etc.
   */
  default void logModules() {}

  /**
   * Applies the specified control request to this swerve drivetrain.
   *
   * @param request Request to apply.
   */
  default void setControl(SwerveRequest request) {}

  /**
   * Register the specified lambda to be executed whenever our SwerveDriveState function is updated
   * in our odometry thread.
   *
   * <p>It is imperative that this function is cheap, as it will be executed along with the odometry
   * call, and if this takes a long time, it may negatively impact the odometry of this stack.
   *
   * <p>This can also be used for logging data if the function performs logging instead of
   * telemetry. Additionally, the SwerveDriveState object can be cloned and stored for later
   * processing.
   *
   * @param telemetryFunction Function to call for telemetry or logging.
   */
  default void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {}

  /**
   * Changes the neutral mode to use for all modules' drive motors.
   *
   * @param neutralMode The new drive motor neutral mode.
   */
  default void changeNeutralMode(NeutralModeValue neutralMode) {}

  /**
   * Zero's this swerve drive's odometry entirely.
   *
   * <p>This will zero the entire odometry, and place the robot at (0,0).
   */
  default void tareEverything() {}

  /**
   * Resets the rotation of the robot pose to 0 from the {@link
   * SwerveRequest.ForwardPerspectiveValue#OperatorPerspective} perspective. This makes the current
   * orientation of the robot X forward for field-centric maneuvers.
   *
   * <p>This is equivalent to calling {@link #resetRotation} with the operator perspective rotation.
   */
  default void seedFieldCentric() {}

  /**
   * Resets the pose of the robot. The pose should be from the {@link
   * SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   *
   * @param pose Pose to make the current pose.
   */
  default void resetPose(Pose2d pose) {}

  /**
   * Resets the translation of the robot pose without affecting rotation. The translation should be
   * from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   *
   * @param translation Translation to make the current translation.
   */
  default void resetTranslation(Translation2d translation) {}

  /**
   * Resets the rotation of the robot pose without affecting translation. The rotation should be
   * from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   *
   * @param rotation Rotation to make the current rotation.
   */
  default void resetRotation(Rotation2d rotation) {}

  /**
   * Takes the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective direction and
   * treats it as the forward direction for {@link
   * SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
   *
   * <p>If the operator is in the Blue Alliance Station, this should be 0 degrees. If the operator
   * is in the Red Alliance Station, this should be 180 degrees.
   *
   * <p>This does not change the robot pose, which is in the {@link
   * SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective. As a result, the robot pose
   * may need to be reset using {@link #resetPose}.
   *
   * @param fieldDirection Heading indicating which direction is forward from the {@link
   *     SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   */
  default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that you must
   *     use a timestamp with an epoch since system startup (i.e., the epoch of this timestamp is
   *     the same epoch as {@link Utils#getCurrentTimeSeconds}). This means that you should use
   *     {@link Utils#getCurrentTimeSeconds} as your time source or sync the epochs. An FPGA
   *     timestamp can be converted to the correct timebase using {@link Utils#fpgaToCurrentTime}.
   */
  default void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {}

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that you must
   *     use a timestamp with an epoch since system startup (i.e., the epoch of this timestamp is
   *     the same epoch as {@link Utils#getCurrentTimeSeconds}). This means that you should use
   *     {@link Utils#getCurrentTimeSeconds} as your time source or sync the epochs. An FPGA
   *     timestamp can be converted to the correct timebase using {@link Utils#fpgaToCurrentTime}.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  default void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {}

  /** Class used to store the IO values of a CTRE Swerve Drivetrain. */
  class DriveIOInputs extends SwerveDriveState implements LoggableInputs, Cloneable {
    /** Updates this {@link DriveIOInputs} from a {@link SwerveDriveState}. */
    public void fromSwerveDriveState(SwerveDriveState state) {
      this.Pose = state.Pose;
      this.Speeds = state.Speeds;
      this.ModuleStates = state.ModuleStates;
      this.ModuleTargets = state.ModuleTargets;
      this.ModulePositions = state.ModulePositions;
      this.RawHeading = state.RawHeading;
      this.Timestamp = state.Timestamp;
      this.OdometryPeriod = state.OdometryPeriod;
      this.SuccessfulDaqs = state.SuccessfulDaqs;
      this.FailedDaqs = state.FailedDaqs;
    }

    // The methods below would normally be generated by @AutoLog annotation, but since this class
    // extends SwerveDriveState, it doesn't seem to function properly, so instead, we do it
    // manually.
    @Override
    public void toLog(LogTable table) {
      table.put("Pose", Pose);
      table.put("Speeds", Speeds);
      table.put("ModuleStates", ModuleStates);
      table.put("ModuleTargets", ModuleTargets);
      table.put("ModulePositions", ModulePositions);
      table.put("RawHeading", RawHeading);
      table.put("Timestamp", Timestamp);
      table.put("OdometryPeriod", OdometryPeriod);
      table.put("SuccessfulDaqs", SuccessfulDaqs);
      table.put("FailedDaqs", FailedDaqs);
    }

    @Override
    public void fromLog(LogTable table) {
      Pose = table.get("Pose", Pose);
      Speeds = table.get("Speeds", Speeds);
      ModuleStates = table.get("ModuleStates", ModuleStates);
      ModuleTargets = table.get("ModuleTargets", ModuleTargets);
      ModulePositions = table.get("ModulePositions", ModulePositions);
      RawHeading = table.get("RawHeading", RawHeading);
      Timestamp = table.get("Timestamp", Timestamp);
      OdometryPeriod = table.get("OdometryPeriod", OdometryPeriod);
      SuccessfulDaqs = table.get("SuccessfulDaqs", SuccessfulDaqs);
      FailedDaqs = table.get("FailedDaqs", FailedDaqs);
    }

    @Override
    public DriveIOInputs clone() {
      return (DriveIOInputs) super.clone();
    }
  }
}
