/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * A class that represents a {@link SubsystemBase} with a state machine implementation. By default, this state machine should be added to a form of state manager (likely a whole robot state machine), or else inputs will not be updated and states will not be automatically switched.
 *
 * <p> The default behavior of the state machine goes through these steps inside a robot code loop:
 *
 * <p>1. When the current loop is first run, the CommandScheduler will call of the subsystems
 * periodic functions. In the case for a state machine, periodic will update all inputs, and switch
 * to a new state if the current state/objective is finished/reached.
 *
 * <p>2. After all subsystems have had their periodic functions called, commands will run, likely
 * updating the states of the subsystems/state machines.
 *
 * <p>3. When a state is updated in a state machine, it will transition to this new state, typically
 * by doing something like changing the speed of a motor.
 */
public abstract class StateMachine<S extends Enum<S>> extends SubsystemBase {
  private S currentState;
  private double lastTransitionTimestamp = 0.0;

  /**
   * Creates a new subsystem with a state machine implementation.
   *
   * @param initialState The initial/default state of the subsystem.
   * @see StateMachine
   */
  protected StateMachine(S initialState) {
    currentState = initialState;
  }

  public void periodic() {
    update();
  }


  /**
   * Periodic method called by the {@link edu.wpi.first.wpilibj2.command.CommandScheduler} each
   * loop. This will update the inputs of the subsystem and switch it to the next state, if it has a
   * next state and if it is ready.
   */
  public void update() {
    // Update the inputs of this subsystem
    updateInputs();

    // Get and set the next state
    setState(getNextState());
  }

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
   * Returns if the subsystem has fully reached it's current state, if it's goal has been reached.
   * This can be used to check to see if a subsystem has reached the current state's position,
   * speed, etc.
   *
   * <p>By default, this returns true, and will only return other values if overridden.
   *
   * @return If the subsystem is at the state's goal.
   */
  public boolean atGoal() {
    return atGoal(currentState);
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

  /**
   * Return the current state of this subsystem.
   *
   * @return The current state of this subsystem.
   */
  public S getCurrentState() {
    return currentState;
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
   * Function to change the state of the subsystem and start a transition.
   *
   * @param goalState The state to switch to.
   */
  protected void setState(S goalState) {
    if (!(goalState == currentState)) {
      Logger.recordOutput(this.getName() + "/GoalState", goalState);
      currentState = getTransitionState(goalState);
      startTransition();
    }
  }

  /**
   * Perform a transition between two states. This is where most of the logic of the state machine
   * actually exists, and where the user should change the speed, position, and more of the
   * subsystems. This will transition to the assigned current state of the subsystem.
   */
  protected void transition() {}

  /**
   * Update the inputs of this subsystem. This is typically done through an AdvantageKit-style IO
   * class.
   */
  protected void updateInputs() {}

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
   * Start a transition to the new current state. This method will save the current timestamp at the
   * time of the transition, log the new state of the subsystem, and then call {@link #transition()}
   * to actually perform the full transition and set the speed, position, etc., of the subsystem.
   */
  private void startTransition() {
    // Get the timestamp where the transition occurs
    lastTransitionTimestamp = Timer.getTimestamp();

    // Log the new current state
    Logger.recordOutput(this.getName() + "/CurrentState", currentState);

    // Run the actual transition
    transition();
  }
}
