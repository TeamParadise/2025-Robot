/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.drivetrain;

import com.team1165.robot.subsystems.drive.Drive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseProfiled extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> pose;

  private Supplier<TrapezoidProfile.State> previousState;

  private final ProfiledPIDController translationController =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4.5, 6.5));
  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(7.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4, 6));

  public DriveToPoseProfiled(Drive drive, Supplier<Pose2d> pose) {
    this.drive = drive;
    this.pose = pose;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);
  }

  public DriveToPoseProfiled(
      Drive drive, Supplier<Pose2d> pose, Supplier<TrapezoidProfile.State> previousState) {
    this(drive, pose);
    this.previousState = previousState;
  }

  public DriveToPoseProfiled(
      Drive drive,
      Supplier<Pose2d> pose,
      Supplier<TrapezoidProfile.State> previousState,
      TrapezoidProfile.Constraints constraints) {
    this(drive, pose, previousState);
    translationController.setConstraints(constraints);
  }

  @Override
  public void initialize() {
    var currentPose = drive.getPose();
    var fieldChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getSpeeds(), currentPose.getRotation());
    var linearFieldVelocity =
        new Translation2d(
            fieldChassisSpeeds.vxMetersPerSecond, fieldChassisSpeeds.vyMetersPerSecond);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.reset(
        drive.getPose().getRotation().getRadians(), drive.getSpeeds().omegaRadiansPerSecond);

    if (previousState != null) {
      translationController.reset(previousState.get());
    } else {
      translationController.reset(
          currentPose.getTranslation().getDistance(pose.get().getTranslation()),
          Math.min(
              0.0,
              -linearFieldVelocity
                  .rotateBy(
                      pose.get()
                          .getTranslation()
                          .minus(currentPose.getTranslation())
                          .getAngle()
                          .unaryMinus())
                  .getX()));
    }
  }

  @Override
  public void execute() {
    // Get target and current pose
    var targetPose = pose.get();
    var currentPose = drive.getPose();

    // Calculate the current distance away from the target pose
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // Calculate distance and rotation
    double translationVelocityScalar = translationController.calculate(currentDistance, 0.0);
    double rotationVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Convert velocity to be based on the angle of movement
    var translationVelocity =
        new Pose2d(
                Translation2d.kZero,
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(translationVelocityScalar, 0.0, Rotation2d.kZero))
            .getTranslation();

    // Create chassis speeds
    var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translationVelocity.getX(),
            translationVelocity.getY(),
            rotationVelocity,
            currentPose.getRotation());

    // Run chassis speeds
    drive.runRobotSpeeds(chassisSpeeds);

    // Log info
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
    Logger.recordOutput("DriveToPose/ChassisSpeeds", chassisSpeeds);
  }

  public TrapezoidProfile.State getCurrentState() {
    return translationController.getSetpoint();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
