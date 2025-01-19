/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.io;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team1165.robot.subsystems.drive.constants.DriveConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

/**
 * {@link DriveIO} class that implements the CTRE {@link SwerveDrivetrain} class, and makes it
 * partially AdvantageKit compatible. This class will automatically start a simulation thread to
 * update the simulation state of the drivetrain.
 */
public class DriveIOSim extends DriveIOReal {
  private double lastSimTime;
  private Notifier simNotifier = null;

  /**
   * Constructs a {@link DriveIOSim} using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   * @param modules Constants for each specific module.
   */
  public DriveIOSim(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    startSimThread();
  }

  /**
   * Constructs a {@link DriveIOSim} using the specified constants.
   *
   * <p>This constructs the underlying simulated hardware devices, so users should not construct the
   * devices themselves.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module.
   */
  public DriveIOSim(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    startSimThread();
  }

  /**
   * Constructs a {@link DriveIOSim} using the specified constants.
   *
   * <p>This constructs the underlying simulated hardware devices, so users should not construct the
   * devices themselves.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param modules Constants for each specific module.
   */
  public DriveIOSim(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    super(drivetrainConstants, modules);
    startSimThread();
  }

  /** Start the simulation thread to update the simulation state of the {@link SwerveDrivetrain}. */
  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    // Run simulation at a faster rate so PID gains behave more reasonably
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              // Use the measured time delta, get battery voltage from WPILib
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(DriveConstants.simulationLoopPeriod);
  }
}
