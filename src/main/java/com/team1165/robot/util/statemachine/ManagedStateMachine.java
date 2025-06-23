/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

/**
 * A class that represents a managed {@link StateMachine}. A managed state machine is controlled
 * through some form of "State Manager" (as an example, a whole robot state machine).
 *
 * <p>The only difference between this and a {@link StateMachine} is as follows:
 * <li>{@link StateMachine} will call {@link #update()} periodically through the {@link
 *     edu.wpi.first.wpilibj2.command.CommandScheduler}
 * <li>{@link ManagedStateMachine} does not update its inputs periodically, and instead relies on a
 *     "State Manager" to do it.
 */
public abstract class ManagedStateMachine<S extends Enum<S>> extends StateMachine<S> {
  /**
   * Creates a new subsystem with a managed state machine implementation.
   *
   * @param initialState The initial/default state of the subsystem.
   * @see StateMachine
   * @see ManagedStateMachine
   */
  protected ManagedStateMachine(S initialState) {
    super(initialState);
  }

  /**
   * Unused periodic method for a {@link ManagedStateMachine}. To update the managed state machine,
   * call {@link #update()} from some form of external "State Manager"
   */
  @Override
  public void periodic() {}
}
