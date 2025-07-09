/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel;

import com.team1165.robot.subsystems.roller.io.RollerIO;
import com.team1165.robot.subsystems.roller.io.RollerIO.RollerIOInputs;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import com.team1165.robot.util.statemachine.OverridableStateMachine;
import com.team1165.robot.util.statemachine.StateUtils;
import java.util.EnumMap;
import org.littletonrobotics.junction.Logger;

/** State-machine-based Funnel subsystem, powered by two motors. */
public class Funnel extends OverridableStateMachine<FunnelState> {
  private final RollerIO io;
  private final RollerIOInputs inputs = new RollerIOInputs();

  private final EnumMap<FunnelState, LoggedTunableNumber> tunableMap =
      StateUtils.createTunableNumberMap(name + "/Voltages", FunnelState.class);

  public Funnel(RollerIO io) {
    super(FunnelState.IDLE);
    this.io = io;
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  @Override
  protected void transition() {
    io.runVolts(tunableMap.get(getCurrentState()).get());
  }
}
