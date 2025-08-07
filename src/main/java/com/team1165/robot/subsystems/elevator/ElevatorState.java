/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator;

import com.team1165.robot.globalconstants.FieldConstants.Reef;
import com.team1165.robot.util.statemachine.State;
import java.util.OptionalDouble;

/** All the possible states for the {@link Elevator} subsystem. */
public enum ElevatorState implements State {
  /** Basic L1 state. */
  L1(Reef.Level.L1.getElevatorHeight()),
  /** Basic L2 state. */
  L2(Reef.Level.L2.getElevatorHeight()),
  /** Basic L3 state. */
  L3(Reef.Level.L3.getElevatorHeight()),
  /** Basic L4 state, */
  L4(Reef.Level.L4.getElevatorHeight()),
  /** Idle, at base position. */
  IDLE(0.0),
  /** Default intake state to get coral into the scoring mechanism. */
  INTAKE(0.0),
  /** Tells the motors to stop output. */
  STOP(Double.NaN),
  /** State that should slowly push the second stage into the first stage, allowing for zeroing. */
  ZEROING(0.0);

  private final double position;

  ElevatorState(double position) {
    this.position = position;
  }

  @Override
  public OptionalDouble get() {
    return Double.isNaN(position) ? OptionalDouble.empty() : OptionalDouble.of(position);
  }
}
