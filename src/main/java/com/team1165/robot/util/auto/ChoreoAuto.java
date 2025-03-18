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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChoreoAuto {
  private final Time delayBeforeStart;
  private final boolean pushPartner;
  private final AutoSegmentConfig[] segments;

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

  public Command getAutoCommand(AutoRoutine routine) {
    // Get the flipped reef location, if we need to flip (for A, H, I, J, K, L)
    Location flippedOverX = segments[0].reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == segments[0].reefLocation());

    AutoTrajectory scoringTrajectory =
        flipped
            ? routine.trajectory(
                ChoreoUtils.flipOverX(
                    routine.trajectory("SL to " + flippedOverX.name()).getRawTrajectory(),
                    "SL to " + segments[0].reefLocation().name()))
            : routine.trajectory("SL to " + flippedOverX.name());

    AutoTrajectory coralStationTrajectory = buildCoralStation(routine, segments[0]);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                scoringTrajectory.resetOdometry(),
                new WaitCommand(delayBeforeStart),
                scoringTrajectory.cmd()));

    scoringTrajectory
        .done()
        .onTrue(
            new WaitCommand(segments[0].delayAfterScoring()).andThen(coralStationTrajectory.cmd()));
    coralStationTrajectory.done().onTrue(new WaitCommand(segments[0].delayAfterIntake()));

    var lastTraj = coralStationTrajectory;
    for (int i = 1; i < segments.length; i++) {
      var newTraj = buildScoring(routine, segments[i - 1].coralStation(), segments[i]);
      var newCs = buildCoralStation(routine, segments[i]);
      lastTraj.done().onTrue(newTraj.cmd());
      newTraj.done().onTrue(newCs.cmd());
      lastTraj = newCs;
    }

    return routine.cmd();
  }

  public static AutoTrajectory buildScoring(
      AutoRoutine routine,
      CoralStationLocation startingCoralStation,
      AutoSegmentConfig mainSequence) {
    // region Create the scoring trajectory and commands
    // Get the flipped reef location, if we need to flip (for A, H, I, J, K, L)
    Location flippedOverX = mainSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == mainSequence.reefLocation());
    CoralStationLocation flippedStartingCs =
        flipped
            ? (startingCoralStation == CoralStationLocation.RCS
                ? CoralStationLocation.LCS
                : CoralStationLocation.RCS)
            : startingCoralStation;

    AutoTrajectory scoringTrajectory =
        flipped
            ? routine.trajectory(
                ChoreoUtils.flipOverX(
                    routine
                        .trajectory(flippedStartingCs.name() + " to " + flippedOverX.name())
                        .getRawTrajectory(),
                    startingCoralStation.name() + " to " + mainSequence.reefLocation().name()))
            : routine.trajectory(flippedStartingCs.name() + " to " + flippedOverX.name());
    // endregion

    return scoringTrajectory;
  }

  public static AutoTrajectory buildCoralStation(
      AutoRoutine routine, AutoSegmentConfig mainSequence) {
    Location flippedOverX = mainSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == mainSequence.reefLocation());

    // region Create the coral station trajectory and commands
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
    // endregion
  }
}
