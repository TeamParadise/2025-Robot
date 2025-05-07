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

public abstract class StateMachine<S extends Enum<S>> extends SubsystemBase {
  private S currentState;
  private boolean isInitalized = false;
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
   * Return the current state of this subsystem.
   *
   * @return The current state of this subsystem.
   */
  public S getCurrentState() {
    return currentState;
  }

  protected void setState(S state) {
    if (!(state == currentState)) {
      state = currentState;
      startTransition();
    }
  }

  /** Start a transition between two states. This method will log the current timestamp at the time of the transition, log the new state of the subsystem, and then call <code>afterTransition()</code> to actually perform the full transition and set the speed, position, etc., of the subsystem. */
  private void startTransition() {
    lastTransitionTimestamp = Timer.getTimestamp();
    Logger.recordOutput(this.getName() + "/CurrentState", currentState);
  }

  /** Perform a transition between two states. This is where most of the logic of the state machine actually exists, and where the user should change the speed, position, and more of the subsystems. This will transition to the assigned current state of the subsystem. */
  protected void transition() {}
}
