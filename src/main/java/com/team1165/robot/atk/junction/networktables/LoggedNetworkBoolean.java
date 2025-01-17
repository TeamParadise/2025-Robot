/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction.networktables;

import com.team1165.robot.atk.junction.LogTable;
import com.team1165.robot.atk.junction.Logger;
import com.team1165.robot.atk.junction.inputs.LoggableInputs;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Manages a boolean value published to the root table of NT. */
public class LoggedNetworkBoolean extends LoggedNetworkInput {
  private final String key;
  private final BooleanEntry entry;
  private boolean defaultValue = false;
  private boolean value;

  /**
   * Creates a new LoggedNetworkBoolean, for handling a boolean input sent via NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *     "/DashboardInputs/{key}" when logged.
   */
  public LoggedNetworkBoolean(String key) {
    this.key = key;
    this.entry = NetworkTableInstance.getDefault().getBooleanTopic(key).getEntry(false);
    this.value = defaultValue;
    Logger.registerDashboardInput(this);
  }

  /**
   * Creates a new LoggedNetworkBoolean, for handling a boolean input sent via NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedNetworkBoolean(String key, boolean defaultValue) {
    this(key);
    setDefault(defaultValue);
    this.value = defaultValue;
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(boolean defaultValue) {
    this.defaultValue = defaultValue;
    entry.set(entry.get(defaultValue));
  }

  /**
   * Publishes a new value. Note that the value will not be returned by {@link #get()} until the
   * next cycle.
   */
  public void set(boolean value) {
    entry.set(value);
  }

  /** Returns the current value. */
  public boolean get() {
    return value;
  }

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(removeSlash(key), value);
        }

        public void fromLog(LogTable table) {
          value = table.get(removeSlash(key), defaultValue);
        }
      };

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.get(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }
}
