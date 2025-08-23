/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel;

import com.team1165.robot.util.statemachine.State;
import java.util.OptionalDouble;

/** All the possible states for the {@link Funnel} subsystem. */
public enum FunnelState implements State {
  /** Idle, not moving. */
  IDLE(0.0),
  /** Default intake state to get coral into the scoring mechanism. */
  INTAKE(1.8),
  /** Spin forwards. */
  MANUAL_FORWARD(1.5),
  /** Spin backwards. */
  MANUAL_REVERSE(-1.5),
  /** Custom state modified on the fly. Used as an emergency backup or for tuning. */
  CUSTOM_MANUAL(0.0);

  private final double voltage;

  FunnelState(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public OptionalDouble get() {
    return OptionalDouble.of(voltage);
  }
}
