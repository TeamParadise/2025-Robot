/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction.networktables;

public abstract class LoggedNetworkInput {
  public static final String prefix = "NetworkInputs";

  /**
   * Update the current value and save/replay the input. This function should not be called by the
   * user.
   */
  public abstract void periodic();

  /** Removes the leading slash from a key. */
  protected static String removeSlash(String key) {
    if (key.startsWith("/")) {
      return key.substring(1);
    } else {
      return key;
    }
  }
}
