/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

public class Constants {
  /** The running mode of the robot (REAL, SIM, or REPLAY). By default, set to REAL. */
  public static Mode robotMode = Mode.REAL;

  /**
   * Whether or not the robot has some extra tuning values (PID, setpoints, etc) enabled that can be
   * changed through a dashboard. Most subsystems will be forced to use these values, which means
   * normal operation likely will not work.
   */
  public static final boolean tuningMode = true;

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
