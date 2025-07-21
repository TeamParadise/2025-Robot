/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1165.robot.subsystems.drive.constants.DriveConstants.PathConstants;
import com.team1165.robot.subsystems.drive.constants.TunerConstants;
import com.team1165.robot.subsystems.drive.io.DriveIO;
import com.team1165.robot.subsystems.drive.io.DriveIO.DriveIOInputs;
import com.team1165.robot.subsystems.drive.io.DriveIOMapleSim;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drive subsystem that uses the CTRE Swerve Library in order to control an ordinary swerve
 * drivebase/drivetrain. Any features/capabilities will be implemented here, instead of inside the
 * {@link DriveIO} classes.
 */
public class Drive extends SubsystemBase {
  // Create initial variables for DriveIO and DriveIOInputs
  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();

  // Store absolute max velocity and rotational speeds
  private final double absoluteMaxVelocity = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double absoluteMaxRotationalRate =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          / Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontRight.LocationY);

  // Create robot speed SwerveRequest for path following/drive to pose
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  // Create PID controllers for path following and drive to pose
  private final PIDController xController =
      new PIDController(
          PathConstants.translation.kP, PathConstants.translation.kI, PathConstants.translation.kD);
  private final PIDController yController =
      new PIDController(
          PathConstants.translation.kP, PathConstants.translation.kI, PathConstants.translation.kD);
  private final PIDController rotationController =
      new PIDController(
          PathConstants.rotation.kP, PathConstants.rotation.kI, PathConstants.rotation.kD);

  // Create NetworkTables table to post Field2d
  private final Field2d field = new Field2d();

  /**
   * Drive subsystem that uses the CTRE Swerve Library in order to control an ordinary swerve
   * drivebase/drivetrain. Any features/capabilities will be implemented here, instead of inside the
   * {@link DriveIO} classes.
   *
   * @param io The {@link DriveIO} to use for this subsystem.
   */
  public Drive(DriveIO io) {
    // Set the IO variable
    this.io = io;
    // Put the Field2d value onto the dashboard
    SmartDashboard.putData("Field", field);
    // Configure the rotation controller to accept continuous input
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // Update the IO inputs and log them
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    // Log the additional information about the modules
    io.logModules();
    // Update the Field2d object with the new pose
    field.setRobotPose(inputs.Pose);
  }

  // region Automation and path following
  public void runRobotSpeeds(ChassisSpeeds speeds) {
    io.setControl(applyRobotSpeeds.withSpeeds(speeds));
  }

  // endregion

  // region Getters (for rotation, pose, speeds, etc)

  /** Gets the absolute maximum speed/velocity of the robot in meters per second. */
  public double getAbsoluteMaxSpeed() {
    return absoluteMaxVelocity;
  }

  /** Gets the absolute maximum rotational rate of the robot in radians per second. */
  public double getAbsoluteMaxRotationalRate() {
    return absoluteMaxRotationalRate;
  }

  /**
   * Get the current {@link Pose2d} of the drivetrain.
   *
   * @return The current {@link Pose2d} of the drivetrain.
   */
  public Pose2d getPose() {
    return inputs.Pose;
  }

  public Pose2d getSimulationPose() {
    if (io.getClass() == DriveIOMapleSim.class) {
      return ((DriveIOMapleSim) io).getSimulationPose();
    } else {
      return inputs.Pose;
    }
  }

  /** Get the rotation of the robot at a certain timestamp for vision. */
  public Rotation2d getRotation(double timestamp) {
    // Run latency compensation based on the timestamp provided
    return new Rotation2d(
        inputs.Pose.getRotation().getRadians()
            - (inputs.Speeds.omegaRadiansPerSecond * (Timer.getTimestamp() - timestamp)));
  }

  /**
   * Get the current {@link ChassisSpeeds} of the drivetrain.
   *
   * @return The current {@link ChassisSpeeds} of the drivetrain.
   */
  public ChassisSpeeds getSpeeds() {
    return inputs.Speeds;
  }

  // endregion

  // region Default CTRE Methods
  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Applies the specified control request to this swerve drivetrain.
   *
   * @param request Request to apply
   */
  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

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
   * @param telemetryFunction Function to call for telemetry or logging
   */
  public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
    io.registerTelemetry(telemetryFunction);
  }

  /**
   * Changes the neutral mode to use for all modules' drive motors.
   *
   * @param neutralMode The new drive motor neutral mode
   */
  public void changeNeutralMode(NeutralModeValue neutralMode) {
    io.changeNeutralMode(neutralMode);
  }

  /**
   * Zero's this swerve drive's odometry entirely.
   *
   * <p>This will zero the entire odometry, and place the robot at 0,0
   */
  public void tareEverything() {
    io.tareEverything();
  }

  /**
   * Resets the rotation of the robot pose to 0 from the {@link
   * SwerveRequest.ForwardPerspectiveValue#OperatorPerspective} perspective. This makes the current
   * orientation of the robot X forward for field-centric maneuvers.
   *
   * <p>This is equivalent to calling {@link #resetRotation} with the operator perspective rotation.
   */
  public void seedFieldCentric() {
    io.seedFieldCentric();
  }

  /**
   * Resets the pose of the robot. The pose should be from the {@link
   * SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   *
   * @param pose Pose to make the current pose
   */
  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  /**
   * Resets the translation of the robot pose without affecting rotation. The translation should be
   * from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   *
   * @param translation Translation to make the current translation
   */
  public void resetTranslation(Translation2d translation) {
    io.resetTranslation(translation);
  }

  /**
   * Resets the rotation of the robot pose without affecting translation. The rotation should be
   * from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
   *
   * @param rotation Rotation to make the current rotation
   */
  public void resetRotation(Rotation2d rotation) {
    io.resetRotation(rotation);
  }

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
   *     SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective
   */
  public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
    io.setOperatorPerspectiveForward(fieldDirection);
  }

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
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

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
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    io.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  /**
   * Sets the pose estimator's trust in robot odometry. This might be used to change trust in
   * odometry after an impact with the wall or traversing a bump.
   *
   * @param stateStdDevs Standard deviations of the pose estimate. Increase these numbers to trust
   *     your state estimate less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
   *     and radians.
   */
  public void setStateStdDevs(Matrix<N3, N1> stateStdDevs) {
    io.setStateStdDevs(stateStdDevs);
  }
  // endregion
}
