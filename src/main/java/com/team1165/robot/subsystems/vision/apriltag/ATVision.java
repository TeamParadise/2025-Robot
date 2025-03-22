/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag;

import static com.team1165.robot.subsystems.vision.apriltag.constants.ATVisionConstants.*;

import com.ctre.phoenix6.Utils;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIO;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOInputsAutoLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayDeque;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem for a collection of cameras that estimates the pose of the robot with AprilTags
 * placed around the field.
 */
public class ATVision extends SubsystemBase {
  // Alerts to send for each of the cameras if they get disconnected
  private final Alert[] disconnectedAlerts;

  // General IO classes and inputs
  private final ATVisionIO[] io;
  private final ATVisionIOInputsAutoLogged[] inputs;
  private final Transform3d[] cameraTransforms;

  // Consumer of the vision pose, supplier of the robot rotation
  private final ATVisionConsumer globalConsumer;
  private final ATVisionRotationSupplier rotationSupplier;

  /**
   * Creates a new {@link ATVision} with the provided values.
   *
   * @param globalConsumer The pose estimation consumer that all cameras of this subsystem
   *     contribute to.
   * @param robotRotationSupplier The supplier of the robot rotation. Must be able to take in a
   *     timestamp for latency compensation.
   * @param config The configurations of all the cameras of this subsystem.
   */
  public ATVision(
      ATVisionConsumer globalConsumer,
      ATVisionRotationSupplier robotRotationSupplier,
      CameraConfig... config) {
    // Initialize consumer and supplier
    this.globalConsumer = globalConsumer;
    rotationSupplier = robotRotationSupplier;

    // Initialize all arrays for each camera
    disconnectedAlerts = new Alert[config.length];
    io = new ATVisionIO[config.length];
    inputs = new ATVisionIOInputsAutoLogged[config.length];
    cameraTransforms = new Transform3d[config.length];
    for (int i = 0; i < config.length; i++) {
      // Initialize IO and IO inputs
      io[i] = config[i].io();
      inputs[i] = new ATVisionIOInputsAutoLogged();
      // Update inputs once so that we can get the name value from the cameras
      io[i].updateInputs(inputs[i]);

      // Add robot to camera transform to array
      cameraTransforms[i] = config[i].robotToCamera();

      // Activate single-tag trig by default
      io[i].setSingleTagTrig(false);

      // Create the alert that will be sent if the camera is disconnected
      disconnectedAlerts[i] =
          new Alert(
              "The AprilTag camera \"" + inputs[i].name + "\" (ID " + i + ") is disconnected.",
              AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    // Initialize values to be logged from all cameras
    Queue<Pose3d> allTagPoses = new ArrayDeque<>();
    Queue<Pose3d> allRobotPoses = new ArrayDeque<>();
    Queue<Pose3d> allRobotPosesAccepted = new ArrayDeque<>();
    Queue<Pose3d> allRobotPosesRejected = new ArrayDeque<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update current inputs and log the current inputs
      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      Logger.processInputs("AprilTagVision/Camera" + cameraIndex, inputs[cameraIndex]);

      // Update the disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize values to be logged from this camera
      Queue<Pose3d> tagPoses = new ArrayDeque<>();
      Queue<Pose3d> robotPoses = new ArrayDeque<>();
      Queue<Pose3d> robotPosesAccepted = new ArrayDeque<>();
      Queue<Pose3d> robotPosesRejected = new ArrayDeque<>();

      // Add visible tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var poseObservation : inputs[cameraIndex].poseObservations) {
        if (poseObservation.tagCount() > 1) { // MultiTag
          // Calculate robot pose from the camera pose provided
          Pose3d robotPose =
              poseObservation.bestCameraPose().plus(cameraTransforms[cameraIndex].inverse());

          // Check whether to reject pose
          boolean rejectPose =
              Math.abs(robotPose.getZ()) > maxZError // Must have realistic Z coordinate
                  // Must be within the field boundaries
                  || robotPose.getX() < -fieldBorderMargin
                  || robotPose.getX() > aprilTagLayout.getFieldLength() + fieldBorderMargin
                  || robotPose.getY() < -fieldBorderMargin
                  || robotPose.getY() > aprilTagLayout.getFieldWidth() + fieldBorderMargin;

          // Add pose to log
          robotPoses.add(robotPose);
          if (rejectPose) {
            robotPosesRejected.add(robotPose);
          } else {
            robotPosesAccepted.add(robotPose);
          }

          // Skip if rejected
          if (rejectPose) {
            continue;
          }

          // Calculate standard deviations
          double stdDevFactor =
              Math.pow(poseObservation.averageTagDistance(), 2.0) / poseObservation.tagCount();
          double linearStdDev = linearStdDevBaseline * stdDevFactor;
          double angularStdDev = angularStdDevBaseline * stdDevFactor;

          // Send vision observation
          globalConsumer.accept(
              robotPose.toPose2d(),
              Utils.fpgaToCurrentTime(poseObservation.timestamp()),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        } else { // Single tag
          // Get the best and the alternate pose
          Pose3d robotPose1 =
              poseObservation.bestCameraPose().plus(cameraTransforms[cameraIndex].inverse());
          Pose3d robotPose2 =
              poseObservation.alternateCameraPose().plus(cameraTransforms[cameraIndex].inverse());

          // Check ambiguity to see if we can immediately reject the pose
          boolean rejectPose = poseObservation.ambiguity() > maxAmbiguity;

          if (!rejectPose) {
            Pose3d robotPose;
            // Get the current rotation of the robot
            Rotation2d currentRotation = rotationSupplier.getRotation(Timer.getTimestamp());
            if (Math.abs(
                    currentRotation.minus(robotPose1.getRotation().toRotation2d()).getRadians())
                < Math.abs(
                    currentRotation.minus(robotPose2.getRotation().toRotation2d()).getRadians())) {
              robotPose = robotPose1;
            } else {
              robotPose = robotPose2;
            }

            // Check whether to reject pose
            rejectPose =
                Math.abs(robotPose.getZ()) > maxZError // Must have realistic Z coordinate
                    // Must be within the field boundaries
                    || robotPose.getX() < -fieldBorderMargin
                    || robotPose.getX() > aprilTagLayout.getFieldLength() + fieldBorderMargin
                    || robotPose.getY() < -fieldBorderMargin
                    || robotPose.getY() > aprilTagLayout.getFieldWidth() + fieldBorderMargin;

            // Add pose to log
            robotPoses.add(robotPose);
            if (rejectPose) {
              robotPosesRejected.add(robotPose);
            } else {
              robotPosesAccepted.add(robotPose);
            }

            // Skip if rejected
            if (rejectPose) {
              continue;
            }

            // Calculate standard deviations
            double linearStdDev =
                linearStdDevSingleTagBaseline * Math.pow(poseObservation.averageTagDistance(), 2.0);
            double angularStdDev =
                angularStdDevBaseline * Math.pow(poseObservation.averageTagDistance(), 2.0);

            // Send vision observation
            globalConsumer.accept(
                robotPose.toPose2d(),
                Utils.fpgaToCurrentTime(poseObservation.timestamp()),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
          } else {
            robotPoses.add(robotPose1);
            robotPosesRejected.add(robotPose1);
          }
        }
      }

      // Loop over single tag observations
      for (var observation : inputs[cameraIndex].singleTagObservations) {
        // Get the robot to camera transform/offset
        Transform3d robotToCamera = cameraTransforms[cameraIndex];

        // Get the 2D translation from the camera to the tag
        Translation2d camToTagTranslation =
            new Pose3d(Translation3d.kZero, new Rotation3d(0, observation.ty(), observation.tx()))
                .transformBy(
                    new Transform3d(
                        new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(0, robotToCamera.getRotation().getY(), 0))
                .toTranslation2d();

        // Get the rotation of the tag from the camera translation
        Rotation2d camToTagRotation =
            rotationSupplier
                .getRotation(observation.timestamp())
                .plus(
                    robotToCamera
                        .getRotation()
                        .toRotation2d()
                        .plus(camToTagTranslation.getAngle()));

        // Get the 2D tag pose from the AprilTag layout
        var tagPose =
            aprilTagLayout.getTagPose(observation.tagId()).map(Pose3d::toPose2d).orElse(null);
        if (tagPose == null) {
          break;
        }

        // Get the translation from the field to the camera
        Translation2d fieldToCameraTranslation =
            new Pose2d(tagPose.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0, new Rotation2d()))
                .getTranslation();

        // Get the robot pose with the translation
        var robotPose =
            new Pose2d(
                    fieldToCameraTranslation,
                    rotationSupplier
                        .getRotation(observation.timestamp())
                        .plus(robotToCamera.getRotation().toRotation2d()))
                .transformBy(
                    new Transform2d(
                        new Pose2d(
                            robotToCamera.getTranslation().toTranslation2d(),
                            robotToCamera.getRotation().toRotation2d()),
                        Pose2d.kZero));

        var robotPose3d = new Pose3d(robotPose);
        robotPoses.add(robotPose3d);
        robotPosesAccepted.add(robotPose3d);

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.distance(), 2.0);
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = Double.POSITIVE_INFINITY;

        // Send vision observation
        globalConsumer.accept(
            robotPose,
            Utils.fpgaToCurrentTime(observation.timestamp()),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera pose data
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));

      // Add camera data to the all queues
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data from all cameras
    Logger.recordOutput("AprilTagVision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("AprilTagVision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  public void enableSingleTagTrig() {
    for (int i = 0; i < io.length; i++) {
      io[i].setSingleTagTrig(true);
    }
  }

  @FunctionalInterface
  public interface ATVisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @FunctionalInterface
  public interface ATVisionRotationSupplier {
    Rotation2d getRotation(double timestampSeconds);
  }

  public record CameraConfig(ATVisionIO io, Transform3d robotToCamera) {}
}
