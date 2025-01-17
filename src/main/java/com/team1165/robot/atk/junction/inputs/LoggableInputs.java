/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction.inputs;

import com.team1165.robot.atk.junction.LogTable;

/**
 * A set of values which can be logged and replayed (for example, the hardware inputs for a
 * subsystem). Data is stored in LogTable objects.
 */
public interface LoggableInputs {
  /** Updates a LogTable with the data to log. */
  public void toLog(LogTable table);

  /** Updates data based on a LogTable. */
  public void fromLog(LogTable table);
}
