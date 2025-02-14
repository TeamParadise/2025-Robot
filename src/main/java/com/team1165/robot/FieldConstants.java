/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  public static final Pose2d[] centerFaces =
      new Pose2d[6]; // Starting facing the driver station in clockwise order
  public static final Pose2d[] scoringLocations = new Pose2d[12];

  public static final Distance fieldLength = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
  public static final double fieldLengthMeters = fieldLength.in(Units.Meters);
  public static final Distance fieldWidth = Units.Feet.of(26).plus(Units.Inches.of(5));
  public static final double fieldWidthMeters = fieldWidth.in(Units.Meters);
  public static final Distance startingLineX =
      Units.Inches.of(299.438); // Measured from the inside of starting line

  public static final double faceLength = Units.Inches.of(36.792600).in(Units.Meters);
  public static final Translation2d center =
      new Translation2d(Units.Inches.of(176.746).in(Units.Meters), fieldWidthMeters / 2.0);
  public static final double faceToZoneLine =
      Units.Inches.of(12).in(Units.Meters); // Side of the reef to the inside of the reef zone line

  private static class GenericPoses {
    private static final Pose2d reefA =
        new Pose2d(3.1886031999999997, 4.1902126, Rotation2d.fromDegrees(0));
    private static final Pose2d reefB =
        new Pose2d(3.1886031999999997, 3.8615874000000003, Rotation2d.fromDegrees(0));
    private static final Pose2d reefC =
        new Pose2d(3.9812746857618713, 5.234534687050671, Rotation2d.fromDegrees(360 - 60));
    private static final Pose2d reefD =
        new Pose2d(3.6966769142381293, 5.070222087050671, Rotation2d.fromDegrees(360 - 60));
    private static final Pose2d reefE =
        new Pose2d(5.282019885761871, 5.070222087050671, Rotation2d.fromDegrees(360 - 120));
    private static final Pose2d reefF =
        new Pose2d(4.9974221142381285, 5.234534687050671, Rotation2d.fromDegrees(360 - 120));
    private static final Pose2d reefG =
        new Pose2d(5.7900936000000005, 3.8615874, Rotation2d.fromDegrees(360 - 180));
    private static final Pose2d reefH =
        new Pose2d(5.7900936000000005, 4.1902126, Rotation2d.fromDegrees(360 - 180));
    private static final Pose2d reefI =
        new Pose2d(4.9974221142381285, 2.81726531294933, Rotation2d.fromDegrees(120));
    private static final Pose2d reefJ =
        new Pose2d(5.282019885761871, 2.9815779129493296, Rotation2d.fromDegrees(120));
    private static final Pose2d reefK =
        new Pose2d(3.6966769142381293, 2.9815779129493296, Rotation2d.fromDegrees(60));
    private static final Pose2d reefL =
        new Pose2d(3.9812746857618713, 2.81726531294933, Rotation2d.fromDegrees(60));

    public static final Pose2d rightCoralStation =
        new Pose2d(1.5270318162817094, 0.7215220217145688, Rotation2d.fromDegrees(54.011));
    public static final Pose2d leftCoralStation =
        new Pose2d(
            rightCoralStation.getX(),
            fieldWidthMeters - rightCoralStation.getY(),
            Rotation2d.fromRadians(-rightCoralStation.getRotation().getRadians()));
  }

  public static Pose2d[] getAllPoses() {
    System.out.println(GenericPoses.leftCoralStation.getX());
    System.out.println(GenericPoses.leftCoralStation.getY());
    System.out.println(GenericPoses.leftCoralStation.getRotation().getRadians());
    return new Pose2d[] {
      GenericPoses.leftCoralStation,
    };
  }
}
