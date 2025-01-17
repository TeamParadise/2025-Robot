/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction.console;

public interface ConsoleSource extends AutoCloseable {
  /** Reads all console data that has been produced since the last call to this method. */
  public String getNewData();
}
