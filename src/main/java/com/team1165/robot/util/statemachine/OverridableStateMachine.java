/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * A class that represents a {@link SubsystemBase} with an overridable state machine implementation.
 *
 * <p>An overridable state machine can have it's state overridden through {@link #overrideState(S)},
 * which can be useful to add manual controls to a codebase that is using a {@link RobotManager}
 * instance as a central point of control.
 *
 * <p>To make sure overrides work correctly, a standard automation command should only ever require
 * a {@link RobotManager} instance (not the individual subsystems), and an override/manual command
 * should require individual subsystems.
 *
 * @see StateMachine
 * @param <S> All possible states for this state machine.
 */
public abstract class OverridableStateMachine<S extends State> extends StateMachine<S> {
  /** Stores if a state override is currently active. */
  private boolean stateOverrideActive = false;

  /**
   * Creates a new {@link SubsystemBase} with an overridable state machine implementation.
   *
   * @param initialState The initial/default state of the state machine.
   * @see OverridableStateMachine
   */
  protected OverridableStateMachine(S initialState) {
    super(initialState);
  }

  /**
   * Creates a command to override the state of this subsystem. This command will set the
   * currentState to the provided state, and will prevent a {@link RobotManager} instance from
   * changing the state, until this command ends.
   *
   * <p>This command will never end without interruption. Make sure to interrupt it by using a
   * Trigger or calling another override command. If interrupted, it will call the {@link
   * RobotManager} to get the current managed state of the subsystem, and the subsystem will return
   * to that state. Interrupting with another override command will immediately start the override
   * sequence again.
   *
   * <p>If this override should persist until the end of the match or when the robot code is
   * disabled (this could be useful for a form of emergency stop), you can use {@link
   * Command#withInterruptBehavior(InterruptionBehavior)} to ensure that the command cannot be
   * interrupted by other commands. You can still cancel it using {@link
   * CommandScheduler#cancel(Command...)}.
   *
   * @param state The state to override with.
   */
  public Command overrideState(S state) {
    var command =
        Commands.runOnce(
                () -> {
                  setState(state);
                  Logger.recordOutput(name + "StateOverride", stateOverrideActive = true);
                })
            .alongWith(Commands.idle());

    // Run this when the override command is interrupted
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (cmd, interupt) -> {
              Logger.recordOutput(name + "StateOverride", stateOverrideActive = false);
              if (interupt.isEmpty()) { // If it wasn't interrupted by another command
                setState(RobotManager.getManagedState(this));
              }
            });

    return command;
  }

  /**
   * Sets a new state for the subsystem and begins the transition process if an override is not
   * active.
   *
   * @param newState The desired state to transition to.
   */
  @Override
  protected void setState(S newState) {
    if (!stateOverrideActive) {
      super.setState(newState);
    }
  }
}
