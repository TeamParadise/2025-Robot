/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.constants;

/** Constants for CAN bus frequencies, for both CAN FD and CAN 2.0. */
public enum CANFrequency {
  /** Fast frequency for the CAN bus, used for values actually used by the code. */
  FAST(250, 100),
  /** Medium frequency for the CAN bus, used for logged values. */
  MEDIUM(100, 50),
  /** Slow frequency for the CAN bus, used for values like faults. */
  SLOW(25, 10);

  private final double fdFrequency;
  private final double standardFrequency;

  CANFrequency(double fdFrequency, double standardFrequency) {
    this.fdFrequency = fdFrequency;
    this.standardFrequency = standardFrequency;
  }

  /**
   * Get this frequency for the CAN bus, based on whether the bus is CAN FD or CAN 2.0
   *
   * @param canFD True if the bus is CAN FD (CANivore), false if the bus is CAN 2.0 (RIO)
   * @return This frequency for that CAN bus.
   */
  public final double getFrequency(boolean canFD) {
    return canFD ? fdFrequency : standardFrequency;
  }
}
