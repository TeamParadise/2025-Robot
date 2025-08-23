/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel;

import com.team1165.robot.util.statemachine.State;
import java.util.OptionalDouble;

/** All the possible states for the {@link Flywheel} subsystem. */
public enum FlywheelState implements State {
  /** Idle, not moving. */
  IDLE(0.0),
  /** Default intake state to get coral into the scoring mechanism. */
  INTAKE(2.0),
  /** Slow scoring speed. */
  SLOW_SCORE(1.4),
  /** Fast scoring speed. */
  FAST_SCORE(4.0),
  /** Spin forwards. */
  MANUAL_FORWARD(1.5),
  /** Spin backwards. */
  MANUAL_REVERSE(-1.5),
  /** Custom state modified on the fly. Avoid use. */
  CUSTOM_MANUAL(0.0);

  private final double voltage;

  FlywheelState(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public OptionalDouble get() {
    return OptionalDouble.of(voltage);
  }
}
