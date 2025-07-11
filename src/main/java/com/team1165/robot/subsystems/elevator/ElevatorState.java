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
  /** Idle, at base position. */
  IDLE(0.0),
  /** Tells the motors to stop output. */
  STOP(Double.NaN),
  /** State that should push the second stage into the first stage, allowing for zeroing. */
  ZEROING(-0.2),
  /** Default intake state to get coral into the scoring mechanism. */
  INTAKE(0.0),
  /** Basic L1 state. */
  BASIC_L1(Reef.Level.L1.getElevatorHeight()),
  /** Basic L2 state. */
  BASIC_L2(Reef.Level.L2.getElevatorHeight()),
  /** Basic L3 state. */
  BASIC_L3(Reef.Level.L3.getElevatorHeight()),
  /** Basic L4 state, */
  BASIC_L4(Reef.Level.L4.getElevatorHeight()),
  /** Adaptive L1 state that will change based on the pose of the drivetrain. */
  ADAPTIVE_L1(Reef.Level.L1.getElevatorHeight()),
  /** Adaptive L2 state that will change based on the pose of the drivetrain. */
  ADAPTIVE_L2(Reef.Level.L2.getElevatorHeight()),
  /** Adaptive L3 state that will change based on the pose of the drivetrain. */
  ADAPTIVE_L3(Reef.Level.L3.getElevatorHeight()),
  /** Adaptive L4 state that will change based on the pose of the drivetrain. */
  ADAPTIVE_L4(Reef.Level.L4.getElevatorHeight());

  private final double position;

  ElevatorState(double position) {
    this.position = position;
  }

  public OptionalDouble get() {
    return Double.isNaN(position) ? OptionalDouble.of(position) : OptionalDouble.empty();
  }
}
