/*
 * File originally made by: Mechanical Advantage - FRC 6328
 * Copyright (c) 2025 Team 6328 (https://github.com/Mechanical-Advantage)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1165.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/** Commands to control the {@link Drive} subsystem. */
public class DriveCommands {
  // Teleop control constants
  private static final double translationalDeadband = 0.05;
  private static final double rotationDeadband = 0.05;

  // Control requests for CTRE swerve
  private static final SwerveRequest.FieldCentric fieldCentricRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private static final SwerveRequest.RobotCentric robotCentricRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  /**
   * Utility method to get the requested linear velocity from a joystick/thumbstick, adapting for
   * the circularity of the joystick to ensure that we go full speed, even when going in a diagonal.
   *
   * @param x The X position of the joystick.
   * @param y The Y position of the joystick.
   * @param fieldCentric Whether the angle of output should be flipped if the robot is on the red
   *     alliance, used for field-centric driving.
   * @return The adapted linear velocity.
   */
  private static Translation2d getLinearVelocityFromJoysticks(
      double x, double y, boolean fieldCentric) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), translationalDeadband);

    // Return new linear velocity with squared magnitude for more precise control
    return new Translation2d(linearMagnitude * linearMagnitude, 0.0)
        .rotateBy(
            new Rotation2d(
                Math.atan2(x, y)
                    + (fieldCentric
                            && DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red
                        ? Math.PI
                        : 0)));
  }

  /**
   * Default drive command in teleop, where the driver fully controls translation and rotation.
   *
   * @param drive The {@link Drive} subsystem to control.
   * @param xSupplier The requested X speed (typically the negative of the left joystick Y axis).
   * @param ySupplier The requested Y speed (typically the negative of the left joystick X axis).
   * @param rotationSupplier The requested rotation speed (either the negative of the right joystick
   *     X axis OR the left trigger axis minus the right trigger axis).
   * @param fieldCentric Whether to control the robot in a field-centric fashion or a robot-centric
   *     fashion.
   */
  public static Command teleopManualDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      boolean fieldCentric) {
    return drive.run(
        () -> {
          // Get corrected linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), fieldCentric);

          // Apply rotation deadband and
          double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), rotationDeadband);
          // Squaring probably isn't needed for trigger rotation, but uncomment if preferred
          // omega = Math.copySign(omega * omega, omega);

          // Set the control of the drive to our new speeds
          drive.setControl(
              fieldCentric
                  ? fieldCentricRequest
                      .withVelocityX(linearVelocity.getY() * drive.getAbsoluteMaxSpeed())
                      .withVelocityY(linearVelocity.getX() * drive.getAbsoluteMaxSpeed())
                      .withRotationalRate(omega * drive.getAbsoluteMaxRotationalRate())
                  : robotCentricRequest
                      .withVelocityX(linearVelocity.getY() * drive.getAbsoluteMaxSpeed())
                      .withVelocityY(linearVelocity.getX() * drive.getAbsoluteMaxSpeed())
                      .withRotationalRate(omega * drive.getAbsoluteMaxRotationalRate()));
        });
  }
}
