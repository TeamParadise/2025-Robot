/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.globalconstants;

import com.team1165.robot.globalconstants.FieldConstants.Reef.Location;
import com.team1165.robot.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  // region Actual "Field" constants (length, width, starting line)
  public static final Distance fieldLength =
      Units.Feet.of(57).plus(Units.Inches.of(6.0 + 7.0 / 8.0));
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
        new Pose2d(3.6966769142381293, 2.9815779129493296, Rotation2d.fromDegrees(60));
    private static final Pose2d reefD =
        new Pose2d(3.9812746857618713, 2.81726531294933, Rotation2d.fromDegrees(60));
    private static final Pose2d reefE =
        new Pose2d(4.9974221142381285, 2.81726531294933, Rotation2d.fromDegrees(120));
    private static final Pose2d reefF =
        new Pose2d(5.282019885761871, 2.9815779129493296, Rotation2d.fromDegrees(120));
    private static final Pose2d reefG =
        new Pose2d(5.7900936000000005, 3.8615874, Rotation2d.fromDegrees(360 - 180));
    private static final Pose2d reefH =
        new Pose2d(5.7900936000000005, 4.1902126, Rotation2d.fromDegrees(360 - 180));
    private static final Pose2d reefI =
        new Pose2d(5.282019885761871, 5.070222087050671, Rotation2d.fromDegrees(360 - 120));
    private static final Pose2d reefJ =
        new Pose2d(4.9974221142381285, 5.234534687050671, Rotation2d.fromDegrees(360 - 120));
    private static final Pose2d reefK =
        new Pose2d(3.9812746857618713, 5.234534687050671, Rotation2d.fromDegrees(360 - 60));
    private static final Pose2d reefL =
        new Pose2d(3.6966769142381293, 5.070222087050671, Rotation2d.fromDegrees(360 - 60));

    public static final Pose2d rightCoralStation =
        new Pose2d(1.6070318162817094, 0.8015220217145688, Rotation2d.fromDegrees(54.011));
    public static final Pose2d leftCoralStation =
        new Pose2d(
            rightCoralStation.getX(),
            fieldWidthMeters - rightCoralStation.getY(),
            Rotation2d.fromRadians(-rightCoralStation.getRotation().getRadians()));
  }

  /** Elevator heights for scoring on the Reef. */
  private static class ElevatorHeights {
    private static final double l1 = 2.0;
    private static final double l2 = 3.68;
    private static final double l3 = 7.12;
    private static final double l4 = 12.05;
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
    private static final Transform2d reefA = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefB = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefC = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefD = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefE = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefF = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefG = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefH = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefI = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefJ = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefK = new Transform2d(-0.08, 0.0, Rotation2d.kZero);
    private static final Transform2d reefL = new Transform2d(-0.08, 0.0, Rotation2d.kZero);

    // Fudge factors for coral station poses
    private static final Transform2d rightCoralStation =
        new Transform2d(0.0, 0.0, Rotation2d.kZero);
    private static final Transform2d leftCoralStation = new Transform2d(0.0, 0.0, Rotation2d.kZero);
  }

  /** Alliance poses based off the generic poses combined with the fudge factors. */
  private static class AlliancePoses {
    private static class Blue {
      // Reef poses
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

      // Coral station poses
      private static final Pose2d rightCoralStation =
          GenericPoses.rightCoralStation.transformBy(FudgeFactors.rightCoralStation);
      private static final Pose2d leftCoralStation =
          GenericPoses.leftCoralStation.transformBy(FudgeFactors.leftCoralStation);
    }

    private static class Red {
      // Reef poses
      private static final Pose2d reefA = flipPoseAlliance(Blue.reefA);
      private static final Pose2d reefB = flipPoseAlliance(Blue.reefB);
      private static final Pose2d reefC = flipPoseAlliance(Blue.reefC);
      private static final Pose2d reefD = flipPoseAlliance(Blue.reefD);
      private static final Pose2d reefE = flipPoseAlliance(Blue.reefE);
      private static final Pose2d reefF = flipPoseAlliance(Blue.reefF);
      private static final Pose2d reefG = flipPoseAlliance(Blue.reefG);
      private static final Pose2d reefH = flipPoseAlliance(Blue.reefH);
      private static final Pose2d reefI = flipPoseAlliance(Blue.reefI);
      private static final Pose2d reefJ = flipPoseAlliance(Blue.reefJ);
      private static final Pose2d reefK = flipPoseAlliance(Blue.reefK);
      private static final Pose2d reefL = flipPoseAlliance(Blue.reefL);

      // Coral station poses
      private static final Pose2d rightCoralStation = flipPoseAlliance(Blue.rightCoralStation);
      private static final Pose2d leftCoralStation = flipPoseAlliance(Blue.leftCoralStation);
    }

    private static Pose2d flipPoseAlliance(Pose2d pose) {
      return new Pose2d(
          fieldLengthMeters - pose.getX(),
          fieldWidthMeters - pose.getY(),
          pose.getRotation().rotateBy(Rotation2d.kPi));
    }

    private static Pose2d getPose(Location level) {
      boolean flip = AllianceFlipUtil.shouldFlip();

      return switch (level) {
        case A -> flip ? Red.reefA : Blue.reefA;
        case B -> flip ? Red.reefB : Blue.reefB;
        case C -> flip ? Red.reefC : Blue.reefC;
        case D -> flip ? Red.reefD : Blue.reefD;
        case E -> flip ? Red.reefE : Blue.reefE;
        case F -> flip ? Red.reefF : Blue.reefF;
        case G -> flip ? Red.reefG : Blue.reefG;
        case H -> flip ? Red.reefH : Blue.reefH;
        case I -> flip ? Red.reefI : Blue.reefI;
        case J -> flip ? Red.reefJ : Blue.reefJ;
        case K -> flip ? Red.reefK : Blue.reefK;
        case L -> flip ? Red.reefL : Blue.reefL;
      };
    }
  }

  /** Constants for Reef scoring and lineup. */
  public static final class Reef {
    public enum Level {
      L1,
      L2,
      L3,
      L4;

      public double getElevatorHeight() {
        return switch (this) {
          case L1 -> ElevatorHeights.l1;
          case L2 -> ElevatorHeights.l2;
          case L3 -> ElevatorHeights.l3;
          case L4 -> ElevatorHeights.l4;
        };
      }
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
      L;

      public Pose2d getPose() {
        return AlliancePoses.getPose(this);
      }
    }
  }

  /** Simple enum to specify the coral station to score at. */
  public enum CoralStationLocation {
    /** The right coral station FROM THE DRIVER STATION PERSPECTIVE. */
    RCS,
    /** The left coral station FROM THE DRIVER STATION PERSPECTIVE. */
    LCS;

    public Pose2d getPose() {
      return switch (this) {
        case RCS ->
            AllianceFlipUtil.shouldFlip()
                ? AlliancePoses.Red.rightCoralStation
                : AlliancePoses.Blue.rightCoralStation;
        case LCS ->
            AllianceFlipUtil.shouldFlip()
                ? AlliancePoses.Red.leftCoralStation
                : AlliancePoses.Blue.leftCoralStation;
      };
    }
  }
}
