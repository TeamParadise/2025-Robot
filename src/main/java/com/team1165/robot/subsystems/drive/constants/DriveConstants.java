/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team1165.robot.subsystems.drive.io.DriveIOMapleSim.MapleSimConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class DriveConstants {
  /** Drivetrain constants (taken directly from TunerConstants). */
  public static final SwerveDrivetrainConstants drivetrainConstants =
      TunerConstants.DrivetrainConstants;

  /**
   * Array of module constants for easy creation of the Drive subsystem. Taken from TunerConstants.
   */
  public static final SwerveModuleConstants<?, ?, ?>[] moduleConstants = {
    TunerConstants.FrontLeft,
    TunerConstants.FrontRight,
    TunerConstants.BackLeft,
    TunerConstants.BackRight
  };

  /** Constants for autonomous driving and line up of the robot. */
  public static final class PathConstants {
    public static final PIDConstants translation = new PIDConstants(10, 0, 0);
    public static final PIDConstants rotation = new PIDConstants(7, 0, 0);
    public static final PPHolonomicDriveController ppDriveController =
        new PPHolonomicDriveController(translation, rotation);
  }

  /**
   * Details about the robot used for autonomous driving (mainly with PathPlanner) and accurate
   * simulation (using maple-sim).
   */
  public static final RobotConfig robotConfig =
      new RobotConfig(
          Pounds.of(130), // Robot weight
          KilogramSquareMeters.of(3.47942549), // Robot MOI
          new ModuleConfig(
              // Wheel radius (grabbed from TunerConstants)
              Meters.of(TunerConstants.FrontLeft.WheelRadius),
              // Max speed of drivetrain (grabbed from TunerConstants)
              TunerConstants.kSpeedAt12Volts,
              // Wheel COF
              1.7,
              // Drive motor and gear reduction
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              // Drive current limit (grabbed from TunerConstants)
              Amps.of(TunerConstants.FrontLeft.SlipCurrent),
              // Number of drive motors
              1),
          // Locations of the modules (all grabbed from TunerConstants)
          new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          new Translation2d(
              TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
          new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          new Translation2d(
              TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));

  /**
   * Details about the robot used for accurate simulation. Some of these values are taken from the
   * RobotConfig.
   */
  public static final MapleSimConfig simConfig =
      new MapleSimConfig(
          Seconds.of(0.002), // Loop period
          Kilograms.of(robotConfig.massKG), // Robot mass (from robot config)
          Inches.of(36), // Robot width with bumpers in X direction
          Inches.of(36), // Robot width with bumpers in Y direction
          robotConfig.moduleConfig.driveMotor.withReduction(
              1 / TunerConstants.FrontLeft.DriveMotorGearRatio), // Drive motor
          DCMotor.getFalcon500(1), // Turn motor
          robotConfig.moduleConfig.wheelCOF // Wheel COF (from robot config)
          );
}
