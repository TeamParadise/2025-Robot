/*
 * File originally made by: The Cobra Commanders - FRC 498
 * Copyright (c) 2025 Team 498 (https://github.com/cobracommanders)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * A class that represents a {@link SubsystemBase} with a state machine implementation.
 *
 * <p>A state machine goes through these steps inside a CommandScheduler loop:
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
  private double lastTransitionTimestamp = Timer.getTimestamp();

  /**
   * Creates a new state machine subsystem.
   *
   * @param initialState The initial/default state of the subsystem.
   */
  protected StateMachine(S initialState) {
    currentState = initialState;
  }

  /**
   * Periodic method called by the {@link edu.wpi.first.wpilibj2.command.CommandScheduler} each
   * loop. This will update the inputs of the subsystem and switch it to the next state, if it has a
   * next state and if it is ready.
   */
  @Override
  public void periodic() {
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
    return Commands.waitUntil(() -> this.currentState == goalState);
  }

  public Command waitUntilGoalReached() {
    return Commands.waitUntil(this::atGoal);
  }

  /**
   * Return the current state of this subsystem.
   *
   * @return The current state of this subsystem.
   */
  public S getCurrentState() {
    return currentState;
  }

  /** Returns if the subsystem has fully reached it's current state, if it's goal has been reached. This can be used to check to see if a subsystem has reached the current state's position, speed, etc.
   *
   * <p>By default, this returns true, and will only return other values if overridden.
   * @return If the subsystem is at the state's goal.
   */
  protected boolean atGoal() {
    return true;
  }

  protected S getNextState() {
    return currentState;
  }

  protected void setState(S state) {
    if (!(state == currentState)) {
      currentState = state;
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
   * Start a transition to the new current state. This method will save the current timestamp at the
   * time of the transition, log the new state of the subsystem, and then call {@link #transition()}
   * to actually perform the full transition and set the speed, position, etc., of the subsystem.
   */
  private void startTransition() {
    // Get the timestamp where the transition occurs
    lastTransitionTimestamp = Timer.getTimestamp();
    // Log the new state
    Logger.recordOutput(this.getName() + "/CurrentState", currentState);

    // Run the actual transition
    transition();
  }
}
