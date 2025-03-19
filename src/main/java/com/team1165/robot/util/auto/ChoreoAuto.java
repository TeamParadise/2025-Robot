/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team1165.robot.FieldConstants.*;
import com.team1165.robot.FieldConstants.Reef.Location;
import com.team1165.robot.commands.AutoScore;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.flywheels.Flywheels;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChoreoAuto {
  private final Time delayBeforeStart;
  private final boolean pushPartner;
  private final AutoSegmentConfig[] segments;
  private static final double translationToleranceBeforeScoring = 0.2;
  private static final double rotationToleranceBeforeScoring = 0.78539816339;

  public ChoreoAuto(Time delayBeforeStart, boolean pushPartner, AutoSegmentConfig... segments) {
    this.delayBeforeStart = delayBeforeStart;
    this.pushPartner = pushPartner;
    this.segments = segments;
  }

  public ChoreoAuto(Time delayBeforeStart, AutoSegmentConfig... segments) {
    this(delayBeforeStart, false, segments);
  }

  public ChoreoAuto(boolean pushPartner, AutoSegmentConfig... segments) {
    this(Seconds.zero(), pushPartner, segments);
  }

  public ChoreoAuto(AutoSegmentConfig... segments) {
    this(Seconds.zero(), false, segments);
  }

  public Command getAutoCommand(Drive drive, Elevator elevator, Flywheels flywheels) {
    if (segments.length != 0) {
      AutoRoutine routine =
          drive.getAutoFactory().newRoutine("Starting at " + segments[0].reefLocation());

      // Get the flipped reef location, if we need to flip (for A, H, I, J, K, L)
      Location flippedOverX = segments[0].reefLocation().getFlippedReef();
      boolean flipped = !(flippedOverX == segments[0].reefLocation());

      // region Create trajectories and initialize first trajectory
      AutoTrajectory scoringTrajectory =
          flipped
              ? routine.trajectory(
                  ChoreoUtils.flipOverX(
                      routine
                          .trajectory("SL to " + flippedOverX.name() + (pushPartner ? " Push" : ""))
                          .getRawTrajectory(),
                      "SL to " + segments[0].reefLocation().name() + (pushPartner ? " Push" : "")))
              : routine.trajectory("SL to " + flippedOverX.name() + (pushPartner ? " Push" : ""));

      AutoTrajectory coralStationTrajectory = buildCoralStation(routine, segments[0]);

      routine
          .active()
          .onTrue(
              Commands.sequence(
                  scoringTrajectory.resetOdometry(),
                  new WaitCommand(delayBeforeStart),
                  scoringTrajectory.cmd()));
      // endregion

      // region Run scoring commands
      var finalPose = scoringTrajectory.getFinalPose();
      var autoScore =
          new AutoScore(
              segments[0].reefLocation(), segments[0].reefLevel(), drive, elevator, flywheels);

      finalPose.ifPresent(
          pose2d ->
              scoringTrajectory
                  .atPose(pose2d, translationToleranceBeforeScoring, rotationToleranceBeforeScoring)
                  .onTrue(autoScore));

      scoringTrajectory
          .done()
          .onTrue(
              new WaitCommand(segments[0].delayAfterScoring())
                  .andThen(coralStationTrajectory.cmd()));
      coralStationTrajectory.done().onTrue(new WaitCommand(segments[0].delayAfterIntake()));

      var lastTraj = coralStationTrajectory;
      for (int i = 1; i < segments.length; i++) {
        var newTraj = buildScoring(routine, segments[i - 1].coralStation(), segments[i]);
        var newCs = buildCoralStation(routine, segments[i]);
        lastTraj
            .done()
            .onTrue(new WaitCommand(segments[i - 1].delayAfterIntake()).andThen(newTraj.cmd()));
        newTraj
            .done()
            .onTrue(new WaitCommand(segments[i].delayAfterScoring()).andThen(newCs.cmd()));
        lastTraj = newCs;
      }

      return routine.cmd();
    } else {
      return Commands.none();
    }
  }

  public static AutoTrajectory buildScoring(
      AutoRoutine routine,
      CoralStationLocation startingCoralStation,
      AutoSegmentConfig mainSequence) {
    // Get the flipped reef location, if we need to flip (for A, H, I, J, K, L)
    Location flippedOverX = mainSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == mainSequence.reefLocation());

    // Get the starting coral station, and flip if needed
    CoralStationLocation flippedStartingCs =
        flipped
            ? (startingCoralStation == CoralStationLocation.RCS
                ? CoralStationLocation.LCS
                : CoralStationLocation.RCS)
            : startingCoralStation;

    // Create and return trajectory
    return flipped
        ? routine.trajectory(
            ChoreoUtils.flipOverX(
                routine
                    .trajectory(flippedStartingCs.name() + " to " + flippedOverX.name())
                    .getRawTrajectory(),
                startingCoralStation.name() + " to " + mainSequence.reefLocation().name()))
        : routine.trajectory(flippedStartingCs.name() + " to " + flippedOverX.name());
  }

  public static AutoTrajectory buildCoralStation(
      AutoRoutine routine, AutoSegmentConfig mainSequence) {
    Location flippedOverX = mainSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == mainSequence.reefLocation());

    // Get the flipped position of the coral station, if needed
    CoralStationLocation flippedCs =
        flipped
            ? (mainSequence.coralStation() == CoralStationLocation.RCS
                ? CoralStationLocation.LCS
                : CoralStationLocation.RCS)
            : mainSequence.coralStation();

    return flipped
        ? routine.trajectory(
            ChoreoUtils.flipOverX(
                routine
                    .trajectory(flippedOverX.name() + " to " + flippedCs.name())
                    .getRawTrajectory(),
                mainSequence.reefLocation().name() + " to " + mainSequence.coralStation()))
        : routine.trajectory(flippedOverX.name() + " to " + flippedCs.name());
  }
}
