/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
 *   <li>Whenever a state is updated using {@link #setState(S)}, the state machine attempts to *
 *       transition to the new state immediately. If a direct transition is not possible, a *
 *       "transition" state (determined by {@link #getTransitionState(S)}) is set as the current *
 *       state instead. This evaluation happens instantly upon calling {@link #setState(S)}, not *
 *       during the next periodic loop, allowing the controls to be immediately updated.
 * </ol>
 */
public abstract class StateMachine<S extends State> extends SubsystemBase {
  /** The current state that the subsystem is in. */
  private S currentState;

  /** Stores if a goal override is currently active. */
  private boolean goalOverrideActive = false;

  /** Stores the value of a goal override. */
  private boolean goalOverrideValue = false;

  /** Stores if a state override is currently active. */
  private boolean stateOverrideActive = false;

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
  }

  // region Default methods that are typically not overridden (mostly public)
  /**
   * Returns whether the subsystem has fully reached its current state, i.e., whether its goal has
   * been met. This can be used to determine if the subsystem has achieved the desired position,
   * speed, or other criteria defined by the current state.
   *
   * <p>By default, this always returns {@code true}, and the {@link #atGoal(S)} method can be
   * overridden to provide more meaningful checks (like if the subsystem is in range of the goal
   * position or speed).
   *
   * @return If the subsystem is at the goal defined by the current state.
   */
  public boolean atGoal() {
    return atGoal(currentState);
  }

  /**
   * Returns the current state that this subsystem is in.
   *
   * @return The current state of this subsystem.
   */
  public S getCurrentState() {
    return currentState;
  }

  /**
   * Sets a new goal state for the subsystem and begins the transition process.
   *
   * @param newState The desired state to transition to.
   */
  protected void setState(S newState) {
    // Only attempt transition if the new goal state is not the current/goal state
    if (newState != currentState && newState != this.goalState) {
      // Set the goal state and find the new current state
      goalState = newState;
      currentState = getTransitionState(newState);

      // Log the current state and the goal state
      Logger.recordOutput(this.getName() + "/CurrentState", currentState);
      Logger.recordOutput(this.getName() + "/GoalState", goalState);

      // Record the transition time and perform the transition
      lastTransitionTimestamp = Timer.getTimestamp();
      transition();
    }
  }

  /**
   * Periodic method called by the {@link edu.wpi.first.wpilibj2.command.CommandScheduler} each
   * loop, that calls the {@link #update()} method. If you don't want this to be called periodically
   * by the scheduler, use a {@link ManagedStateMachine} instead or override this method.
   */
  public void periodic() {
    update();
  }

  /**
   * Returns if the current state has been active for longer than the specified duration. Useful for
   * timeout logic during state transitions.
   *
   * @param duration The timeout duration (in seconds) to use.
   * @return Whether the current state has been active longer than the given duration.
   */
  public boolean timeout(double duration) {
    return (Timer.getFPGATimestamp() - lastTransitionTimestamp) > duration;
  }

  // endregion

  // region Methods that are typically overridden (mostly protected)

  /**
   * Method that will update the inputs of the subsystem and switch it to the next state if one is
   * available.
   */
  public final void update() {
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
   * Performs a transition to the new current state. This method contains most of the state
   * machine's logic and is where users should implement changes to subsystem behavior—such as
   * adjusting speed, position, and other parameters.
   */
  protected abstract void transition();

  /**
   * Update the inputs of this subsystem. This is typically done through an AdvantageKit-style IO
   * class.
   */
  protected void updateInputs() {}
  ;

  // endregion
  // region Commands

  /**
   * Creates a command to override the state of this subsystem. This command will set the
   * currentState to the provided state, and will prevent a {@link RobotManager} instance from
   * changing the state, until this command ends.
   *
   * <p>This command will never end without interruption. Make sure to interrupt it using a Trigger,
   * or by calling another override command. If interrupted, it will call the {@link RobotManager}
   * to get the current managed state of the subsystem, and the subsystem will return to that state.
   * Interrupting with another override command will immediately start the override sequence again.
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
    var command = Commands.runOnce(
            () -> {
              setState(state);
              Logger.recordOutput(this.getName() + "StateOverride", stateOverrideActive = true);
            })
        .alongWith(Commands.idle());

    // Run this when the override command is interrupted
    CommandScheduler.getInstance().onCommandInterrupt(
        (cmd, interupt) -> {
          Logger.recordOutput(this.getName() + "StateOverride", stateOverrideActive = false);
          if (interupt.isEmpty()) { // If it wasn't interrupted by another (presumably override) cmd
            setState(RobotManager.getManagedState(this));
          }
        }
    );

    return command;
  }

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
   * Creates a command that finishes when the subsystem reaches any one of the given states.
   *
   * @param states A set of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  public Command waitForStates(Set<S> states) {
    return Commands.waitUntil(() -> states.contains(currentState));
  }

  /**
   * Creates a
   *
   * @param state
   * @return
   */
  public Command waitUntilGoalReached(S state) {
    return Commands.waitUntil(() -> atGoal(state));
  }

  // endregion
}
