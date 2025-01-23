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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Manages a number value published to the root table of NT. */
public class LoggedNetworkNumber extends LoggedNetworkInput {
  private final String key;
  private final DoubleEntry entry;
  private double defaultValue = 0.0;
  private double value;

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *     "/DashboardInputs/{key}" when logged.
   */
  public LoggedNetworkNumber(String key) {
    this.key = key;
    this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0);
    this.value = defaultValue;
    Logger.registerDashboardInput(this);
  }

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedNetworkNumber(String key, double defaultValue) {
    this(key);
    setDefault(defaultValue);
    this.value = defaultValue;
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    entry.set(entry.get(defaultValue));
  }

  /**
   * Publishes a new value. Note that the value will not be returned by {@link #get()} until the
   * next cycle.
   */
  public void set(double value) {
    entry.set(value);
  }

  /** Returns the current value. */
  public double get() {
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
