/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  /** The running mode of the robot (REAL, SIM, or REPLAY). By default, set to REAL. */
  public static final double loopPeriodSecs = 0.02;

  public static Mode robotMode = Mode.REAL;
  public static boolean disableHAL = false;
  private static RobotType robotType = RobotType.COMPBOT;

  /** The possible modes that a robot can run in. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    COMPBOT
  }

  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static void disableHAL() {
    disableHAL = true;
  }
}
