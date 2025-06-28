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

/**
 * A class that represents a {@link SubsystemBase} with an overridable state machine implementation.
 *
 * <p>This state machine can also work with goal states, allowing the subsystem to report whether it
 * is within a certain threshold of its target setpoint. This is useful for automation—such as
 * initiating movement, scoring, or other actions—once the subsystem has reached its final goal.
 *
 * <p>You must implement a proper method (e.g., {@code atGoal()}) to evaluate whether the subsystem
 * is at its goal. This is not provided by default, as different subsystems may require different
 * tolerances or logic (for example, estimating the goal of a drive base will require multiple
 * tolerance values, compared to most other linear or spinning mechanisms).
 *
 * @see StateMachine
 * @see OverridableStateMachine
 * @param <S> All possible states for this state machine.
 */
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

  /**
   * Creates a command to enable the goal override system, forcing the subsystem to report a
   * different goal status (reached or not reached) than what it would normally report, until this
   * command ends.
   *
   * <p>This command will never end without interruption. Make sure to interrupt it by using a
   * Trigger or calling another override command. If interrupted, it'll disable the goal override
   * and trying to get the goal will return its true status.
   *
   * @param goalValue Whether the goal should report as reached (true) or not reached (false).
   */
  public Command goalOverrideCommand(boolean goalValue) {
    return Commands.runOnce(() -> enableGoalOverride(goalValue))
        .alongWith(Commands.idle())
        .finallyDo(this::disableGoalOverride);
  }

  /** Disable the goal override system, causing the subsystem to report its real status. */
  public void disableGoalOverride() {
    Logger.recordOutput(this.getName() + "GoalOverrideActive", goalOverrideActive = false);
    Logger.recordOutput(this.getName() + "GoalOverrideValue", goalOverrideValue = false);
  }

  /**
   * Enable the goal override system, forcing the subsystem to report a different goal status
   * (reached or not reached) than what it would normally report.
   *
   * @param goalValue Whether the goal should report as reached (true) or not reached (false).
   */
  public void enableGoalOverride(boolean goalValue) {
    Logger.recordOutput(this.getName() + "GoalOverrideActive", goalOverrideActive = true);
    Logger.recordOutput(this.getName() + "GoalOverrideValue", goalOverrideValue = goalValue);
  }

  /**
   * Get whether the goal override is currently active.
   *
   * @return If the goal override is currently active.
   */
  public boolean getGoalOverrideActive() {
    return goalOverrideActive;
  }

  /**
   * Get the current value of the goal override.
   *
   * @return The current value of the goal override.
   */
  public boolean getGoalOverrideValue() {
    return goalOverrideValue;
  }
}
