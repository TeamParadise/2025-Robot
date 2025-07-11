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

  public Elevator(ElevatorIO io) {
    super(ElevatorState.IDLE);
    this.io = io;
  }

  public double getRealPosition() {
    return inputs.primaryMotor.position;
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  @Override
  protected void transition() {
    switch (getCurrentState()) {
      case STOP -> io.stop();
      case ADAPTIVE_L1, ADAPTIVE_L2, ADAPTIVE_L3, ADAPTIVE_L4 -> {
        // TODO: Add adaptive code
        break;
      }
      default -> {
        if (tunableMap.containsKey(getCurrentState())) {
          io.runPosition(tunableMap.get(getCurrentState()).get());
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
