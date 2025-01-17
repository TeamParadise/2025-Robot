/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction.networktables;

/**
 * Manages a string value published to the "SmartDashboard" table of NT.
 *
 * @deprecated Use {@link org.littletonrobotics.junction.networktables.LoggedNetworkString} with a
 *     "/SmartDashboard" prefix (e.g. "/SmartDashboard/...")
 */
@Deprecated
public class LoggedDashboardString extends LoggedNetworkString {
  /**
   * Creates a new LoggedDashboardString, for handling a string input sent to the "SmartDashboard"
   * table of NetworkTables.
   *
   * @param key The key for the number, published to the "SmartDashboard" table of NT or
   *     "/DashboardInputs/{key}" when logged.
   */
  public LoggedDashboardString(String key) {
    super("/SmartDashboard/" + key);
  }

  /**
   * Creates a new LoggedDashboardString, for handling a string input sent to the "SmartDashboard"
   * table of NetworkTables.
   *
   * @param key The key for the number, published to the "SmartDashboard" table of NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedDashboardString(String key, String defaultValue) {
    super("/SmartDashboard/" + key, defaultValue);
  }
}
