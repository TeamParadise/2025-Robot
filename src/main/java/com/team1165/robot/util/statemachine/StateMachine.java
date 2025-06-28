/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * A class that represents a {@link SubsystemBase} with a state machine implementation. This state
 * machine periodically updates its inputs via the {@link CommandScheduler}.
 *
 * <p>If you want to have a state machine that supports state overrides (manual control outside a
 * {@link RobotManager}), use a {@link OverridableStateMachine}.
 *
 * <p>The default behavior of the state machine proceeds through the following steps during each
 * command-based robot code loop:
 *
 * <ol>
 *   <li>When the {@link CommandScheduler} runs, it calls each subsystem's {@link #periodic()}
 *       method in an arbitrary order. For a state machine, this {@link #periodic()} method updates
 *       inputs.
 *   <li>After all subsystem periodic methods have run, scheduled commands are executed, which may
 *       update the states of subsystems or state machines.
 *   <li>Whenever a state is updated using {@link #setState(S)}, the state machine attempts to
 *       transition to the new state immediately. This evaluation happens instantly upon calling
 *       {@link #setState(S)}, not during the next periodic loop, allowing the controls to be
 *       immediately updated.
 * </ol>
 *
 * @param <S> All possible states for this state machine.
 */
public abstract class StateMachine<S extends Enum<S> & State> extends SubsystemBase {
  /** The name of this subsystem. */
  protected final String name;
  /** The current state that the subsystem is in. */
  private S currentState;
  /** The last time that a state change occurred. */
  private double lastStateChangeTimestamp = 0.0;

  /**
   * Creates a new {@link SubsystemBase} with a state machine implementation.
   *
   * @param initialState The initial/default state of the state machine.
   * @see StateMachine
   */
  protected StateMachine(S initialState) {
    name = getName();
    setState(initialState);
  }

  // region Commands

  /**
   * Creates a command that ends once this subsystem is in the given state.
   *
   * @param state The state to wait for.
   * @return A command that ends once the current state equals the given state.
   */
  public Command waitForState(S state) {
    return Commands.waitUntil(() -> currentState == state);
  }

  /**
   * Creates a command that ends once this subsystem is in any of the given states.
   *
   * @param states A set of the states to wait for.
   * @return A command that ends once this subsystem is in any of the given states.
   */
  public Command waitForStates(Set<S> states) {
    return Commands.waitUntil(() -> states.contains(currentState));
  }

  // endregion

  // region Default methods that are typically not overridden

  /**
   * Returns the current state that this subsystem is in.
   *
   * @return The current state of this subsystem.
   */
  public S getCurrentState() {
    return currentState;
  }

  /**
   * Returns the last time that a state change occurred.
   *
   * @return The last time a state change occurred.
   */
  public double getLastStateChangeTimestamp() {
    return lastStateChangeTimestamp;
  }

  /**
   * Returns whether the subsystem is currently in the provided state.
   *
   * @return If the subsystem is currently in the provided state.
   */
  public boolean inState(S state) {
    return state == currentState;
  }

  /**
   * Periodic method called by the {@link edu.wpi.first.wpilibj2.command.CommandScheduler} each
   * loop, that calls the {@link #update()} method.
   */
  public void periodic() {
    update();
  }

  /**
   * Sets a new state for the subsystem and begins the transition process.
   *
   * @param newState The desired state to transition to.
   */
  protected void setState(S newState) {
    // Only attempt transition if the new state is not equal to the current state
    if (newState != currentState) {
      // Log the new current state
      Logger.recordOutput(name + "/CurrentState", (currentState = newState).toString());

      // Record the last state change time
      lastStateChangeTimestamp = Timer.getTimestamp();
    }
    // Perform a transition
    transition();
  }

  /**
   * Returns if the current state has been active for longer than the specified duration. Useful for
   * timeout logic during state transitions.
   *
   * @param duration The timeout duration (in seconds) to use.
   * @return Whether the current state has been active longer than the given duration.
   */
  public boolean timeout(double duration) {
    return (Timer.getTimestamp() - lastStateChangeTimestamp) > duration;
  }

  // endregion

  // region Methods that are typically overridden

  /**
   * Performs a transition to the new current state. This method contains most of the state
   * machine's logic and is where users should implement changes to subsystem behavior — such as
   * adjusting speed, position, and other parameters.
   */
  protected abstract void transition();

  /**
   * Method that will update the inputs of the subsystem, typically done through an
   * AdvantageKit-style IO class.
   */
  protected void update() {}

  // endregion
}
