/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import com.team1165.robot.subsystems.roller.flywheel.Flywheel;
import com.team1165.robot.subsystems.roller.flywheel.FlywheelState;
import com.team1165.robot.subsystems.roller.funnel.Funnel;
import com.team1165.robot.subsystems.roller.funnel.FunnelState;
import com.team1165.robot.util.statemachine.RobotManager;
import com.team1165.robot.util.statemachine.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class OdysseusManager extends RobotManager<OdysseusState> {
  private final Funnel funnel;
  private final Flywheel flywheel;

  /**
   * Creates a new {@link RobotManager} to manage a collection of robot subsystems.
   *
   * @param initialState The initial/default state of the manager.
   * @see StateMachine
   */
  protected OdysseusManager(OdysseusState initialState, Funnel funnel, Flywheel flywheel) {
    super(initialState);
    this.funnel = funnel;
    this.flywheel = flywheel;
  }

  public Command stateCommand(OdysseusState state) {
    return Commands.runOnce(() -> setState(state));
  }

  @Override
  protected void transition() {
    switch (getCurrentState()) {
      case L1, L2, L3, L4, IDLE, ZERO_ELEVATOR:
        setSubsystemState(flywheel, FlywheelState.IDLE);
        setSubsystemState(funnel, FunnelState.IDLE);
        break;
      case INTAKE:
        setSubsystemState(flywheel, FlywheelState.INTAKE);
        setSubsystemState(funnel, FunnelState.INTAKE);
        break;
      case SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4:
        setSubsystemState(flywheel, FlywheelState.FAST_SCORE);
        setSubsystemState(funnel, FunnelState.IDLE);
    }
  }
}
