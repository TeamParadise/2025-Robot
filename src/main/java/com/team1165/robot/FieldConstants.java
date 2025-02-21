/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  // region Actual "Field" constants (length, width, starting line)
  public static final Distance fieldLength = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
  public static final double fieldLengthMeters = fieldLength.in(Units.Meters);
  public static final Distance fieldWidth = Units.Feet.of(26).plus(Units.Inches.of(5));
  public static final double fieldWidthMeters = fieldWidth.in(Units.Meters);
  public static final Distance startingLineX =
      Units.Inches.of(299.438); // Measured from the inside of starting line

  // endregion

  /** Generic poses without any "fudge" factors or alliance switching. */
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

  /**
   * Fudge factors for changing constants based on differences between fields.
   *
   * <p>X value (first value) - Increase to move FORWARDS (robot-centric), decrease to move
   * BACKWARDS (robot-centric)
   *
   * <p>Y value (second value) - Increase to move LEFT (robot-centric), decrease to move RIGHT
   * (robot-centric)
   */
  private static class FudgeFactors {
    // Fudge factors for reef scoring sides
    private static final Transform2d reefA = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefB = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefC = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefD = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefE = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefF = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefG = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefH = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefI = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefJ = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefK = new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d reefL = new Transform2d(0.0, 0.0, Rotation2d.kZero);

    // Fudge factors for coral station poses
    private static final Transform2d rightCoralStation =
        new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d leftCoralStation = new Transform2d(0.0, 0.0, Rotation2d.kZero);
  }

  /** Alliance poses based off the generic poses combined with the fudge factors. */
  private static class AlliancePoses {
    private static class Blue {
      private static final Pose2d reefA = GenericPoses.reefA.transformBy(FudgeFactors.reefA);
      private static final Pose2d reefB = GenericPoses.reefB.transformBy(FudgeFactors.reefB);
      private static final Pose2d reefC = GenericPoses.reefC.transformBy(FudgeFactors.reefC);
      private static final Pose2d reefD = GenericPoses.reefD.transformBy(FudgeFactors.reefD);
      private static final Pose2d reefE = GenericPoses.reefE.transformBy(FudgeFactors.reefE);
      private static final Pose2d reefF = GenericPoses.reefF.transformBy(FudgeFactors.reefF);
      private static final Pose2d reefG = GenericPoses.reefG.transformBy(FudgeFactors.reefG);
      private static final Pose2d reefH = GenericPoses.reefH.transformBy(FudgeFactors.reefH);
      private static final Pose2d reefI = GenericPoses.reefI.transformBy(FudgeFactors.reefI);
      private static final Pose2d reefJ = GenericPoses.reefJ.transformBy(FudgeFactors.reefJ);
      private static final Pose2d reefK = GenericPoses.reefK.transformBy(FudgeFactors.reefK);
      private static final Pose2d reefL = GenericPoses.reefL.transformBy(FudgeFactors.reefL);
    }

    private static class Red {

    }
  }

  public static final class Reef {
    public enum Level {
      L1,
      L2,
      L3,
      L4;
    }

    public enum Location {
      A,
      B,
      C,
      D,
      E,
      F,
      G,
      H,
      I,
      J,
      K,
      L
    }
  }
}
