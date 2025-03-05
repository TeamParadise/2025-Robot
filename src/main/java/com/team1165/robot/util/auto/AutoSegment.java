/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import com.team1165.robot.FieldConstants.*;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;

/** An auto segment represents a segment of an auto, mainly consisting of a reef scoring location/level to score on and a coral station to go to after scoring, if wanted. Also can include delays between actions. */
public class AutoSegment {
  private final Reef.Location reefLocation;
  private final Reef.Level reefLevel;
  private final Time delayAfterScoring;
  private final CoralStationLocation coralStation;
  private final Time delayAfterCoralStation;
}
