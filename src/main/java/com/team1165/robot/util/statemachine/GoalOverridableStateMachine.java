/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GoalOverridableStateMachine<S extends State>
    extends OverridableStateMachine<S> {
  /** Stores if a goal override is currently active. */
  private boolean goalOverrideActive = false;

  /** Stores the value of a goal override. */
  private boolean goalOverrideValue = false;

  /**
   * Creates a new {@link SubsystemBase} with an overridable state machine implementation.
   *
   * @param initialState The initial/default state of the state machine.
   * @see StateMachine
   */
  protected GoalOverridableStateMachine(S initialState) {
    super(initialState);
  }

  protected void enableGoalOverride(boolean goalValue) {
    Logger.recordOutput(this.getName() + "GoalOverrideActive", goalOverrideActive = true);
    Logger.recordOutput(this.getName() + "GoalOverrideValue", goalOverrideValue = goalValue);
  }

  protected void disableGoalOverride() {
    Logger.recordOutput(this.getName() + "GoalOverrideActive", goalOverrideActive = false);
    Logger.recordOutput(this.getName() + "GoalOverrideValue", goalOverrideValue = false);
  }

  protected Command goalOverrideCommand(boolean goalValue) {
    return Commands.runOnce(() -> enableGoalOverride(goalValue))
        .alongWith(Commands.idle())
        .finallyDo(this::disableGoalOverride);
  }
}
