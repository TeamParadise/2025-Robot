/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  public static final Distance fieldLength = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
  public static final double fieldLengthMeters = fieldLength.in(Units.Meters);
  public static final Distance fieldWidth = Units.Feet.of(26).plus(Units.Inches.of(5));
  public static final double fieldWidthMeters = fieldWidth.in(Units.Meters);
  public static final Distance startingLineX =
      Units.Inches.of(299.438); // Measured from the inside of starting line

  private static class GenericPoses {
    private static final Pose2d reefA = new Pose2d(2.860, 4.187, Rotation2d.fromDegrees(0));
    private static final Pose2d reefB = new Pose2d(2.860, 3.857, Rotation2d.fromDegrees(0));
    private static final Pose2d reefC = new Pose2d(3.527, 2.694, Rotation2d.fromDegrees(60));
    private static final Pose2d reefD = new Pose2d(3.813, 2.535, Rotation2d.fromDegrees(60));
    private static final Pose2d reefE = new Pose2d(5.160, 2.529, Rotation2d.fromDegrees(120));
    private static final Pose2d reefF = new Pose2d(5.445, 2.694, Rotation2d.fromDegrees(120));
    private static final Pose2d reefG = new Pose2d(6.119, 3.857, Rotation2d.fromDegrees(180));
    private static final Pose2d reefH = new Pose2d(6.119, 4.187, Rotation2d.fromDegrees(180));
    private static final Pose2d reefI = new Pose2d(5.452, 5.343, Rotation2d.fromDegrees(-120));
    private static final Pose2d reefJ = new Pose2d(5.166, 5.527, Rotation2d.fromDegrees(-120));
    private static final Pose2d reefK = new Pose2d(3.826, 5.508, Rotation2d.fromDegrees(-60));
    private static final Pose2d reefL = new Pose2d(3.534, 5.368, Rotation2d.fromDegrees(-60));
  }
}
