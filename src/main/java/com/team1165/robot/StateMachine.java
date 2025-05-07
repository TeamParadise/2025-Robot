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

  /** Periodic method called by the {@link edu.wpi.first.wpilibj2.command.CommandScheduler} each loop. This will update the inputs of the subsystem and switch it to the next state, if it has a next state and if it is ready. */
  @Override
  public void periodic() {
    // Update the inputs of this subsystem
    updateInputs();

    // Get and set the next state
    setState(getNextState());
  }

  /**
   * Return the current state of this subsystem.
   *
   * @return The current state of this subsystem.
   */
  public S getCurrentState() {
    return currentState;
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
   * Start a transition between two states. This method will log the current timestamp at the time
   * of the transition, log the new state of the subsystem, and then call <code>afterTransition()
   * </code> to actually perform the full transition and set the speed, position, etc., of the
   * subsystem.
   */
  private void startTransition() {
    lastTransitionTimestamp = Timer.getTimestamp();
    Logger.recordOutput(this.getName() + "/CurrentState", currentState);
  }

  /**
   * Perform a transition between two states. This is where most of the logic of the state machine
   * actually exists, and where the user should change the speed, position, and more of the
   * subsystems. This will transition to the assigned current state of the subsystem.
   */
  protected void transition() {}

  /** Update the inputs of this subsystem. This is typically done through an AdvantageKit-style IO class. */
  protected void updateInputs() {}
}
