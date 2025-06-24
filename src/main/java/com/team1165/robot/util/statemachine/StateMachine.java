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
 * A class that represents a {@link SubsystemBase} with a state machine implementation. By default,
 * this state machine periodically updates its inputs and current state via the {@link
 * CommandScheduler}. If input updates need to be manually managed (e.g., in a time-based robot or a
 * robot-wide state machine), use a {@link ManagedStateMachine}.
 *
 * <p>The default behavior of the state machine proceeds through the following steps during each
 * command-based robot code loop:
 *
 * <ol>
 *   <li>When the {@link CommandScheduler} runs, it calls each subsystem's {@link #periodic()}
 *       method in an arbitrary order. For a state machine, this {@link #periodic()} method updates
 *       inputs and transitions to a new state if the current state has completed or its goal has
 *       been reached. This is most useful when transitioning between primary states automatically
 *       after a transition goal is reached.
 *   <li>After all subsystem periodic methods have run, scheduled commands are executed, which may
 *       update the states of subsystems or state machines.
 *   <li>Whenever a state is updated using {@link #setState(S)}, the state machine attempts to
 *       transition to the new state immediately. If a direct transition is not possible, a
 *       "transition" state (determined by {@link #getTransitionState(S)}) is set as the current
 *       state instead. This evaluation happens instantly upon calling {@link #setState(S)}, not
 *       during the next periodic loop, allowing the controls to be immediately updated.
 * </ol>
 */
public abstract class StateMachine<S extends Enum<S>> extends SubsystemBase {
  /** The current state that the subsystem is in. */
  private S currentState;

  /** The goal staet that the subsystem is trying to reach. */
  private S goalState;

  /** The last time that a state transition was performed. */
  private double lastTransitionTimestamp = 0.0;

  /**
   * Creates a new {@link SubsystemBase} with a state machine implementation.
   *
   * @param initialState The initial/default state of the state machine.
   * @see StateMachine
   */
  protected StateMachine(S initialState) {
    currentState = initialState;
    goalState = initialState;
  }

  // region Public methods
  /**
   * Returns whether the subsystem has fully reached its current state—i.e., whether its goal has
   * been met. This can be used to determine if the subsystem has achieved the desired position,
   * speed, or other criteria defined by the current state.
   *
   * <p>By default, this always returns {@code true}, and should be overridden to reflect meaningful
   * state completion checks.
   *
   * @return If the subsystem is at the goal defined by the current state.
   */
  public boolean atGoal() {
    return atGoal(currentState);
  }

  /**
   * Return the current state of this subsystem.
   *
   * @return The current state of this subsystem.
   */
  public S getCurrentState() {
    return currentState;
  }

  /**
   * Periodic method called by the {@link edu.wpi.first.wpilibj2.command.CommandScheduler} each
   * loop, that calls the {@link #update()} method. If you don't want this to be called periodically
   * by the scheduler, use a {@link ManagedStateMachine} instead.
   */
  public void periodic() {
    update();
  }

  /**
   * Checks if the current state has been in for longer than the given duration. Used for having
   * timeout logic in state transitions.
   *
   * @param duration The timeout duration (in seconds) to use.
   * @return Whether the current state has been active for longer than the given duration.
   */
  public boolean timeout(double duration) {
    var currentStateDuration = Timer.getFPGATimestamp() - lastTransitionTimestamp;

    return currentStateDuration > duration;
  }

  /**
   * Method that will update the inputs of the subsystem and switch it to the next state if one is
   * available.
   */
  public void update() {
    // Update the inputs of this subsystem
    updateInputs();

    // Get and set the next state
    setState(getNextState());
  }

  /**
   * Creates a command that finishes once this subsystem is in the given state.
   *
   * @param goalState The state to wait for.
   * @return A command that finishes once the current state is equal to the goal state.
   */
  public Command waitForState(S goalState) {
    return Commands.waitUntil(() -> currentState == goalState);
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates A set of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  public Command waitForStates(Set<S> goalStates) {
    return Commands.waitUntil(() -> goalStates.contains(currentState));
  }

  public Command waitUntilGoalReached(S goalState) {
    return Commands.waitUntil(() -> currentState == goalState && atGoal());
  }

  // endregion

  // region Protected methods (these are the ones that should be overridden in an implementation
  /**
   * Check to see if the state machine has reached the "goal" of the specified state. This will
   * typically be used to check to see if the subsystem is within a specific tolerance of the
   * state's goal position or speed. By default, this will return true if the current state is equal
   * to the goal state, and should be overridden for extra functionality.
   *
   * @param goalState The state to check if the subsystem is in tolerance of.
   * @return If the subsystem is within tolerance of the current state.
   */
  protected boolean atGoal(S goalState) {
    return goalState == currentState;
  }

  /**
   * Get the next state of the subsystem, if the subsystem is meant to transition between different
   * states on it's own. By default, this will just return the current state.
   *
   * @return THe next state of the subsystem if it exists, otherwise, the curren state.
   */
  protected S getNextState() {
    return currentState;
  }

  protected S getTransitionState(S goalState) {
    return goalState;
  }

  /**
   * Sets a new goal state for the subsystem and begins the transition process.
   *
   * @param newState The desired state to transition to.
   */
  protected void setState(S newState) {
    // Only attempt transition if the new goal state is not the current/goal state
    if (!(newState == currentState | newState == this.goalState)) {
      // Set the goal state and find the new current state
      goalState = newState;
      currentState = getTransitionState(newState);

      // Log the goal state and the current state
      Logger.recordOutput(this.getName() + "/CurrentState", currentState);
      Logger.recordOutput(this.getName() + "/GoalState", goalState);

      // Record the transition time and perform the transition
      lastTransitionTimestamp = Timer.getTimestamp();
      transition();
    }
  }

  /**
   * Performs a transition to the new current state. This method contains most of the state
   * machine's logic and is where users should implement changes to subsystem behavior—such as
   * adjusting speed, position, and other parameters.
   */
  protected void transition() {}

  /**
   * Update the inputs of this subsystem. This is typically done through an AdvantageKit-style IO
   * class.
   */
  protected void updateInputs() {}

  // endregion
}
