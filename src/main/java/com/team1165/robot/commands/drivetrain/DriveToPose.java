/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.drivetrain;

import com.team1165.robot.subsystems.drive.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> pose;

  private final PIDController translationController = new PIDController(4.0, 0.0, 0.0);
  private final PIDController rotationController = new PIDController(7.0, 0.0, 0.0);

  public DriveToPose(Drive drive, Supplier<Pose2d> pose) {
    this.drive = drive;
    this.pose = pose;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.reset();
    translationController.reset();
    translationController.setSetpoint(0.0);
  }

  @Override
  public void execute() {
    var targetPose = pose.get();
    var currentPose = drive.getPose();

    double translationVelocityScalar =
        translationController.calculate(
            currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    double rotationVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

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
