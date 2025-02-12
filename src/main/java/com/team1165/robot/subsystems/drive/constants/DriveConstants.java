/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class DriveConstants {

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

  /** How often the simulation thread should be run. */
  public static final double simulationLoopPeriod = 0.005;
}
