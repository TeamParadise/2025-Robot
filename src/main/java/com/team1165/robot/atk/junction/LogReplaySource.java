/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction;

/** Provides a stream of log entries to be fed back to the robot code during simulation. */
public interface LogReplaySource {

  /**
   * Called before the logging system begins reporting data. This should be used to connect to
   * files, find network devices, start threads, etc.
   */
  public default void start() {}
  ;

  /**
   * Called when the code shuts down cleanly. Note that this will NOT be called when the robot is
   * powered off.
   */
  public default void end() {}
  ;

  /**
   * Called every loop cycle to get the next set of data.
   *
   * @param table A reference to the current data table, to be updated with new data (including a
   *     timestamp).
   * @return A boolean indicating whether the replay should continue.
   */
  public boolean updateTable(LogTable table);
}
