/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel.sensor;

import org.littletonrobotics.junction.AutoLog;

/** A hardware interface/implementation layer for a sensor (beam break/distance sensor style) for the {@link com.team1165.robot.subsystems.roller.flywheel.Flywheel} subsystem. */
public interface SensorIO {
  /** Class used to store the IO values of a sensor. */
  @AutoLog
  class SensorIOInputs {
    public boolean connected = false;
    public double distance = 0.0;
  }

  /**
   * Updates a {@link SensorIOInputs} instance with the latest updates from this {@link SensorIO}.
   *
   * @param inputs A {@link SensorIOInputs} instance to update.
   */
  default void updateInputs(SensorIOInputs inputs) {}
}
