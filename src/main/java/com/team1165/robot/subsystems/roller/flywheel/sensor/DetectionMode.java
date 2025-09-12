/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel.sensor;

public enum DetectionMode {
  /** Detection using current, only works if the wheels are actively spinning. */
  CURRENT,
  /** Detection using a distance sensor, like a CANrange. */
  DISTANCE_SENSOR
}
