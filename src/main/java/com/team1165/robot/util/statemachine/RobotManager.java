/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

public abstract class RobotManager<S extends Enum<S> & State> extends StateMachine<S> {
  /**
   * Creates a new {@link RobotManager} to manage a collection of robot subsystems.
   *
   * @param initialState The initial/default state of the manager.
   * @see StateMachine
   */
  protected RobotManager(S initialState) {
    super(initialState);

    // TODO: Test this solution compared to just always setting the managed state in terms of time
    /*
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (cmd, interrupt) -> {
              if (interrupt.isPresent()) {
                var interruptRequirements = interrupt.get().getRequirements();
                for (Subsystem subsystem : cmd.getRequirements()) {
                  if ((!interruptRequirements.contains(subsystem))
                      & subsystem instanceof OverridableStateMachine) {
                    ((OverridableStateMachine<?>) subsystem).setManagedState();
                  }
                }
              } else {
                for (Subsystem subsystem : cmd.getRequirements()) {
                  if (subsystem instanceof OverridableStateMachine) {
                    ((OverridableStateMachine<?>) subsystem).setManagedState();
                  }
                }
              }
            });
     */
  }

  protected <T extends Enum<T> & State> void setSubsystemState(
      StateMachine<T> subsystem, T newState) {
    subsystem.setState(newState);
  }
}
