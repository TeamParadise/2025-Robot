/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import com.team1165.robot.subsystems.roller.funnel.Funnel;
import com.team1165.robot.subsystems.roller.funnel.FunnelState;
import com.team1165.robot.util.statemachine.RobotManager;
import com.team1165.robot.util.statemachine.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class OdysseusManager extends RobotManager<FunnelState> {
  private final Funnel funnel;

  /**
   * Creates a new {@link RobotManager} to manage a collection of robot subsystems.
   *
   * @param initialState The initial/default state of the manager.
   * @see StateMachine
   */
  protected OdysseusManager(FunnelState initialState, Funnel funnel) {
    super(initialState);
    this.funnel = funnel;
  }

  public Command stateCommand(FunnelState state) {
    return Commands.runOnce(() -> setState(state));
  }

  @Override
  protected void transition() {
    setSubsystemState(funnel, getCurrentState());
  }
}
