/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import com.team1165.robot.util.statemachine.State;
import java.util.OptionalDouble;

public enum OdysseusState implements State {
  IDLE,
  INTAKE,
  L1,
  SCORE_L1,
  L2,
  SCORE_L2,
  L3,
  SCORE_L3,
  L4,
  SCORE_L4,
  ZERO_ELEVATOR;

  public OptionalDouble get() {
    return OptionalDouble.empty();
  }
}
