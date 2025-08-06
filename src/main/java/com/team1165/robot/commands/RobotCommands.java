/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.OdysseusManager;
import com.team1165.robot.globalconstants.FieldConstants.Reef.Level;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

public class RobotCommands {
  // TODO: Fix this class.

  public static Command setLevelState(OdysseusManager robot, Supplier<Level> level) {
    //    OdysseusState state =
    //        switch (level.get()) {
    //          case L1 -> OdysseusState.L1;
    //          case L2 -> OdysseusState.L2;
    //          case L3 -> OdysseusState.L3;
    //          case L4 -> OdysseusState.L4;
    //        };
    //
    //    return robot.stateCommand(state);
    return Commands.none();
  }

  public static Command setScoreLevelState(OdysseusManager robot, Supplier<Level> level) {
    //    OdysseusState state =
    //        switch (level.get()) {
    //          case L1 -> OdysseusState.SCORE_L1;
    //          case L2 -> OdysseusState.SCORE_L2;
    //          case L3 -> OdysseusState.SCORE_L3;
    //          case L4 -> OdysseusState.SCORE_L4;
    //        };
    //
    //    return robot.stateCommand(state);
    return Commands.none();
  }
}
