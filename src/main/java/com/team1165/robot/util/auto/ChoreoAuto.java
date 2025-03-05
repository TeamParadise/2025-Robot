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

  public AutoRoutine buildAuto(AutoRoutine routine, boolean resetOdometry) {
    // Set up first auto segment manually
    AutoTrajectory firstTraj = routine.trajectory()
  }

  public static AutoTrajectory buildSegment(AutoRoutine routine, AutoSegmentConfig autoSequence) {
    // Get the flipped reef location, if we need to flip (for A, H, I, J, K, L)
    Reef.Location flippedOverX = autoSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == autoSequence.reefLocation());
    CoralStationLocation flippedCS =
        flipped ? (autoSequence.coralStation() == CoralStationLocation.RCS ? CoralStationLocation.LCS : CoralStationLocation.RCS) : autoSequence.coralStation();

    scoringTrajectory = routine.trajectory(new Trajectory<SampleType>())
  }
}
