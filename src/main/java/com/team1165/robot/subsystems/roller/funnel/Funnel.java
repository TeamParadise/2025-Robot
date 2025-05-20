/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel;

import com.team1165.robot.StateMachine;
import com.team1165.robot.subsystems.roller.funnel.constants.FunnelConstants;
import com.team1165.robot.subsystems.roller.io.RollerIO;
import com.team1165.robot.subsystems.roller.io.RollerIO.RollerIOInputs;
import com.team1165.robot.util.logging.LoggedTunableNumber;

public class Funnel extends StateMachine<FunnelState> {
  private final RollerIO io;
  private final RollerIOInputs inputs = new RollerIOInputs();

  private final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber("Funnel/Speeds/Intake", FunnelConstants.Voltages.intake);
  private final LoggedTunableNumber manualForwardVoltage =
      new LoggedTunableNumber(
          "Funnel/Speeds/ManualForward", FunnelConstants.Voltages.manualForward);
  private final LoggedTunableNumber manualReverseVoltage =
      new LoggedTunableNumber(
          "Funnel/Speeds/ManualReverse", FunnelConstants.Voltages.manualReverse);

  public Funnel(RollerIO io) {
    super(FunnelState.IDLE);
    this.io = io;
  }

  @Override
  protected void updateInputs() {
    io.updateInputs(inputs);
  }

  @Override
  protected void transition() {
    switch (getCurrentState()) {
      case IDLE:
        io.runVolts(0.0);
      case INTAKE:
        io.runVolts(intakeVoltage.get());
      case MANUAL_FORWARD:
        io.runVolts(manualForwardVoltage.get());
      case MANUAL_REVERSE:
        io.runVolts(manualReverseVoltage.get());
    }
  }
}
