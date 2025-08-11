/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import com.team1165.robot.globalconstants.FieldConstants.*;

/**
 * An auto segment represents a segment of an auto, mainly consisting of a reef scoring
 * location/level to score on and a coral station to go to after scoring, if wanted. Also can
 * include delays between actions.
 */
public record AutoSegmentConfig(
    Reef.Location reefLocation, Reef.Level reefLevel, CoralStationLocation coralStation) {}
;
