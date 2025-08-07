/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.elevator.ElevatorState;
import com.team1165.robot.subsystems.roller.flywheel.Flywheel;
import com.team1165.robot.subsystems.roller.flywheel.FlywheelState;
import com.team1165.robot.subsystems.roller.funnel.Funnel;
import com.team1165.robot.subsystems.roller.funnel.FunnelState;
import com.team1165.robot.util.statemachine.RobotManager;
import com.team1165.robot.util.statemachine.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

public class OdysseusManager extends RobotManager<OdysseusState> {
  private final Elevator elevator;
  private final Flywheel flywheel;
  private final Funnel funnel;

  /**
   * Creates a new {@link RobotManager} to manage a collection of robot subsystems.
   *
   * @param initialState The initial/default state of the manager.
   * @see StateMachine
   */
  protected OdysseusManager(
      OdysseusState initialState, Elevator elevator, Flywheel flywheel, Funnel funnel) {
    super(initialState);
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.funnel = funnel;
  }

  public double getFlywheelCurrent() {
    // TODO: Maybe goals should be used for this?
    return flywheel.getSupplyCurrent();
  }

  public boolean getElevatorAtGoal(double tolerance) {
    return elevator.atGoal(tolerance);
  }

  public void setState(OdysseusState state) {
    super.setState(state);
  }

  public Command stateCommand(OdysseusState state) {
    return Commands.runOnce(() -> setState(state), this);
  }

  public Command stateSupplierCommand(Supplier<OdysseusState> stateSupplier) {
    return Commands.runOnce(() -> setState(stateSupplier.get()), this);
  }

  @Override
  protected void transition() {
    switch (getCurrentState()) {
        // TODO: Maybe add a method to ElevatorState to get the specified state for a specific reef
        // TODO: level, to prevent having to copy and paste code over and over?
      case IDLE -> {
        setSubsystemState(elevator, ElevatorState.IDLE);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.IDLE);
      }
      case INTAKE -> {
        setSubsystemState(elevator, ElevatorState.INTAKE);
        setSubsystemState(funnel, FunnelState.INTAKE);
        setSubsystemState(flywheel, FlywheelState.INTAKE);
      }
      case L1 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L1);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.IDLE);
      }
      case SCORE_L1 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L1);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.FAST_SCORE);
      }
      case L2 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L2);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.IDLE);
      }
      case SCORE_L2 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L2);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.FAST_SCORE);
      }
      case L3 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L3);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.IDLE);
      }
      case SCORE_L3 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L3);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.FAST_SCORE);
      }
      case L4 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L4);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.IDLE);
      }
      case SCORE_L4 -> {
        setSubsystemState(elevator, ElevatorState.BASIC_L4);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.FAST_SCORE);
      }
      case ZERO_ELEVATOR -> {
        setSubsystemState(elevator, ElevatorState.ZEROING);
        setSubsystemState(funnel, FunnelState.IDLE);
        setSubsystemState(flywheel, FlywheelState.IDLE);
      }
    }
  }
}
