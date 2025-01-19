/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.io;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team1165.robot.subsystems.drive.constants.TunerConstants.TunerSwerveDrivetrain;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

/**
 * {@link DriveIO} class that implements the CTRE {@link SwerveDrivetrain} class, and makes it
 * partially AdvantageKit compatible.
 */
public class DriveIOReal extends TunerSwerveDrivetrain implements DriveIO {
  private final String[] moduleNames = {"Drive/FL/", "Drive/FR/", "Drive/BL/", "Drive/BR/"};

  /**
   * Constructs a {@link DriveIOReal} using the specified constants.
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
  public DriveIOReal(
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
  }

  /**
   * Constructs a {@link DriveIOReal} using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module.
   */
  public DriveIOReal(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
  }

  /**
   * Constructs a {@link DriveIOReal} using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param modules Constants for each specific module.
   */
  public DriveIOReal(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    super(drivetrainConstants, modules);
  }

  /**
   * Updates a {@link DriveIOInputs} instance with the latest updates from this {@link DriveIO}.
   *
   * @param inputs A {@link DriveIOInputs} instance to update.
   */
  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Update the inputs passed in with the current SwerveDriveState.
    inputs.fromSwerveDriveState(getState());
  }

  /**
   * Logs specific additional information about the swerve modules, like temperature, applied
   * output, current, etc.
   */
  @Override
  public void logModules() {
    SwerveModule<?, ?, ?>[] modules = getModules();
    for (int i = 0; i < modules.length; i++) {
      // Log drive motor applied voltage
      Logger.recordOutput(
          moduleNames[i] + "DriveMotor/AppliedVolts",
          modules[i].getDriveMotor().getMotorVoltage().getValue());
      // Log drive motor connection status
      Logger.recordOutput(
          moduleNames[i] + "DriveMotor/Connected",
          modules[i].getDriveMotor().getConnectedMotor().getValue() != ConnectedMotorValue.Unknown);
      // Log drive motor supply current
      Logger.recordOutput(
          moduleNames[i] + "DriveMotor/SupplyCurrent",
          modules[i].getDriveMotor().getSupplyCurrent().getValue());
      // Log drive motor temperature (specifically in Celsius, otherwise, defaults to Kelvin)
      Logger.recordOutput(
          moduleNames[i] + "DriveMotor/Temperature",
          modules[i].getDriveMotor().getDeviceTemp().getValue().in(Celsius));

      // Log steer motor applied voltage
      Logger.recordOutput(
          moduleNames[i] + "SteerMotor/AppliedVolts",
          modules[i].getSteerMotor().getMotorVoltage().getValue());
      // Log steer motor connection status
      Logger.recordOutput(
          moduleNames[i] + "SteerMotor/Connected",
          modules[i].getSteerMotor().getConnectedMotor().getValue() != ConnectedMotorValue.Unknown);
      // Log steer motor supply current
      Logger.recordOutput(
          moduleNames[i] + "SteerMotor/SupplyCurrent",
          modules[i].getSteerMotor().getSupplyCurrent().getValue());
      // Log steer motor temperature (specifically in Celsius, otherwise, defaults to Kelvin)
      Logger.recordOutput(
          moduleNames[i] + "SteerMotor/Temperature",
          modules[i].getSteerMotor().getDeviceTemp().getValue().in(Celsius));
    }
  }

  /**
   * Changes the neutral mode to use for all modules' drive motors.
   *
   * @param neutralMode The new drive motor neutral mode.
   */
  @Override
  public void changeNeutralMode(NeutralModeValue neutralMode) {
    configNeutralMode(neutralMode);
  }
}
