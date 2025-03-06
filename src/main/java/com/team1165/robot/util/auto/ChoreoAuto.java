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
import choreo.trajectory.Trajectory;
import com.team1165.robot.FieldConstants.*;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

  public static Command buildScoring(AutoRoutine routine, CoralStationLocation startingCoralStation, AutoSegmentConfig mainSequence) {
    //region Create the scoring trajectory and commands
    // Get the flipped reef location, if we need to flip (for A, H, I, J, K, L)
    Reef.Location flippedOverX = mainSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == mainSequence.reefLocation());
    CoralStationLocation flippedStartingCs =
        flipped
            ? (startingCoralStation == CoralStationLocation.RCS
            ? CoralStationLocation.LCS
            : CoralStationLocation.RCS)
            : startingCoralStation;

    AutoTrajectory scoringTrajectory =
        routine.trajectory(
            ChoreoUtils.flipOverX(
                routine
                    .trajectory(flippedStartingCs.name() + " to " + flippedOverX.name())
                    .getRawTrajectory(),
                startingCoralStation.name() + " to " + mainSequence.reefLocation().name()));
    //endregion

    //region Create the coral station trajectory and commands
    CoralStationLocation flippedCs =
        flipped
            ? (mainSequence.coralStation() == CoralStationLocation.RCS
            ? CoralStationLocation.LCS
            : CoralStationLocation.RCS)
            : mainSequence.coralStation();

    AutoTrajectory coralStationTrajectory =
        routine.trajectory(
            ChoreoUtils.flipOverX(
                routine
                    .trajectory(flippedOverX.name() + " to " + flippedCs.name())
                    .getRawTrajectory(),
                mainSequence.reefLocation().name() + " to " + mainSequence.coralStation()));
    //endregion

    // Combine command sequences
    // TODO: Check if we would rather do when at a certain pose or time?
    // TODO: Add extra pose correction, like on-the-fly or basic PID to pose.
    // TODO: Add actual auto score and intake command
    Command autoScore = Commands.print("Auto Score!");
    Command autoIntake = Commands.print("Auto Intake!");

    scoringTrajectory.atTime("AutoScore").onTrue(autoScore);
    return scoringTrajectory.cmd();
  }
}
