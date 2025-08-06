/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.constants;

/** Reusable class to determine the robot {@link Mode}, between REAL, SIM, and REPLAY. */
public class RobotMode {
  /** The running mode of the robot (REAL, SIM, or REPLAY). By default, set to REAL. */
  private static Mode robotMode = Mode.REAL;

  /** If the robot mode has been set once, prevent further modifications. */
  private static final boolean modeSet = false;

  /**
   * Get the current robot {@link Mode}.
   *
   * @return The current robot mode.
   */
  public static Mode get() {
    return robotMode;
  }

  /**
   * Set the current robot {@link Mode}. This should only be called once, typically by the Main
   * class.
   *
   * @param mode The robot mode to run in.
   */
  public static void setMode(Mode mode) {
    if (!modeSet) {
      robotMode = mode;
    } else {
      System.err.println(
          "Warning in RobotMode: Attempted to set RobotMode after it has already been set.");
    }
  }

  /** The possible modes that a robot can run in. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
