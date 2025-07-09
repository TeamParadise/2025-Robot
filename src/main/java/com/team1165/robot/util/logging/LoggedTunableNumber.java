/*
 * File originally made by: Mechanical Advantage - FRC 6328
 * Copyright (c) 2025 Team 6328 (https://github.com/Mechanical-Advantage)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.logging;

import com.team1165.robot.globalconstants.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number logged over NetworkTables. This class will get the value from the
 * dashboard if the robot code is in tuning mode, otherwise, it'll return the value specified by the
 * code. Value can only be adjusted from code if the robot code is not in tuning mode.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasValue = false;
  private double value;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Constructs a new LoggedTunableNumber.
   *
   * @param dashboardKey Key on dashboard (under Tuning).
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Constructs a new LoggedTunableNumber with the specified default value. If tuning mode is
   * enabled, this is the only time you can set the value from the code.
   *
   * @param dashboardKey Key on dashboard (under Tuning).
   * @param defaultValue Default value.
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    set(defaultValue);
  }

  /**
   * Set the current value of the number. If the robot is in tuning mode, the value will not be set,
   * unless this is the first time setting the value.
   *
   * @param value The new value to assign.
   */
  public void set(double value) {
    if (!Constants.tuningMode) {
      hasValue = true;
      this.value = value;
    } else if (!hasValue) {
      hasValue = true;
      this.value = value;
      dashboardNumber = new LoggedNetworkNumber(key, value);
    }
  }

  /**
   * Get the current value, from the dashboard in tuning mode, and from the currently set value if
   * not. If no value is currently set, returns 0.0.
   *
   * @return The current value.
   */
  public double get() {
    return (hasValue ? (Constants.tuningMode ? dashboardNumber.get() : value) : 0.0);
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()".
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()".
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method.
   * @param tunableNumbers All tunable numbers to check.
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  /** Runs action if any of the tunableNumbers have changed. */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  /**
   * Get the current value, from the dashboard in tuning mode, and from the currently set value if
   * not. If no value is currently set, returns 0.0.
   *
   * @return The current value.
   */
  @Override
  public double getAsDouble() {
    return get();
  }
}
