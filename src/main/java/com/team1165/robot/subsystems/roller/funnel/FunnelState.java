/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel;

import com.team1165.robot.util.statemachine.State;

/** All the possible states for the {@link Funnel} subsystem. */
public enum FunnelState implements State {
  /** Idle, not moving. */
  IDLE,
  /** Default intake state to get a coral into the scoring mechanism. */
  INTAKE,
  /** Spin forwards. */
  MANUAL_FORWARD,
  /** Spin backwards. */
  MANUAL_REVERSE,
  /** Custom state modified on the fly. Avoid use. */
  CUSTOM_MANUAL
}
