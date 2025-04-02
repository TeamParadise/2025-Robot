/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.rev;

import com.revrobotics.spark.SparkBase.Faults;

/** Record that provides the capability to record the faults of a SPARK motor controller. */
public record SparkFaults(
    boolean temperature,
    boolean sensor,
    boolean gateDriver,
    boolean other) {
  public static SparkFaults getFromFaults(Faults faults) {
    return new SparkFaults(faults.temperature, faults.sensor, faults.gateDriver, faults.other);
  }
}
