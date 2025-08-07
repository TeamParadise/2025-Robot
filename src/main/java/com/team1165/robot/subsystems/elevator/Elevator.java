/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator;

import com.team1165.robot.subsystems.elevator.io.ElevatorIO;
import com.team1165.robot.subsystems.elevator.io.ElevatorIO.ElevatorIOInputs;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import com.team1165.robot.util.statemachine.GoalOverridableStateMachine;
import com.team1165.robot.util.statemachine.StateUtils;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.EnumMap;
import org.littletonrobotics.junction.Logger;

/** State-machine-based Elevator subsystem, powered by two motors. */
public class Elevator extends GoalOverridableStateMachine<ElevatorState> {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  private final EnumMap<ElevatorState, LoggedTunableNumber> tunableMap =
      StateUtils.createTunableNumberMap(name + "/Positions", ElevatorState.class);
  private final LoggedTunableNumber zeroingSpeed =
      new LoggedTunableNumber(name + "/ZeroingSpeed", 0.05);

  private double setpoint = 0.0;

  /**
   * State-machine-based Elevator subsystem, powered by two motors.
   *
   * @param io The {@link ElevatorIO} class to use for this subsystem.
   */
  public Elevator(ElevatorIO io) {
    super(ElevatorState.IDLE);
    this.io = io;
  }

  public boolean atGoal(double tolerance) {
    // TODO: Maybe take the average of both motor positions?
    return getGoalOverrideActive()
        ? getGoalOverrideValue()
        : Math.abs(setpoint - inputs.primaryMotor.position) > tolerance;
  }

  public double getCurrent() {
    return (inputs.primaryMotor.supplyCurrentAmps + inputs.secondaryMotor.supplyCurrentAmps) / 2.0;
  }

  @Override
  protected void update() {
    // If the elevator is in ZEROING, slowly move it down
    if (getCurrentState() == ElevatorState.ZEROING) {
      io.runPosition(setpoint = setpoint - zeroingSpeed.get());
    }

    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  @Override
  protected void transition() {
    switch (getCurrentState()) {
      case STOP -> io.stop();
      default -> {
        if (tunableMap.containsKey(getCurrentState())) {
          setpoint = tunableMap.get(getCurrentState()).get();
          io.runPosition(setpoint);
        } else {
          DriverStation.reportError(
              "Elevator state "
                  + getCurrentState()
                  + " has no provided value or alternatives in the transition() method! Continuing without setting the state.",
              false);
        }
      }
    }
  }
}
