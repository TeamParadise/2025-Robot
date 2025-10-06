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

  private final ProfiledPIDController translationController =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4.5, 6.5));
  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(7.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4, 6));

  private Translation2d lastSetpointTranslation = Translation2d.kZero;

  public DriveToPoseProfiled(Drive drive, Supplier<Pose2d> pose) {
    this.drive = drive;
    this.pose = pose;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);
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

    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    var targetPose = pose.get();
    var currentPose = drive.getPose();

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    translationController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        translationController.getSetpoint().velocity);

    double translationVelocityScalar = translationController.calculate(currentDistance, 0.0);
    double rotationVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(
                    translationController.getSetpoint().position, 0.0, Rotation2d.kZero))
            .getTranslation();

    Logger.recordOutput("DriveToPose/TargetPose", targetPose);

    var translationVelocity =
        new Pose2d(
                Translation2d.kZero,
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(translationVelocityScalar, 0.0, Rotation2d.kZero))
            .getTranslation();

    var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translationVelocity.getX(),
            translationVelocity.getY(),
            rotationVelocity,
            currentPose.getRotation());

    Logger.recordOutput("DriveToPose/ChassisSpeeds", chassisSpeeds);
    drive.runRobotSpeeds(chassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
