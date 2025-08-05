/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.OdysseusManager;
import com.team1165.robot.OdysseusState;
import com.team1165.robot.globalconstants.FieldConstants.Reef;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotCommands {
  public static Command setLevelState(OdysseusManager robot, Reef.Level level) {
    OdysseusState state =
        switch (level) {
          case L1 -> OdysseusState.L1;
          case L2 -> OdysseusState.L2;
          case L3 -> OdysseusState.L3;
          case L4 -> OdysseusState.L4;
        };

    return robot.stateCommand(state);
  }

  public static Command setScoreLevelState(OdysseusManager robot, Reef.Level level) {
    OdysseusState state =
        switch (level) {
          case L1 -> OdysseusState.SCORE_L1;
          case L2 -> OdysseusState.SCORE_L2;
          case L3 -> OdysseusState.SCORE_L3;
          case L4 -> OdysseusState.SCORE_L4;
        };

    return robot.stateCommand(state);
  }
}
