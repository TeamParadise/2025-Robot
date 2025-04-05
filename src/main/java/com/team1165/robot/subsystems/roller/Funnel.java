/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller;

import com.team1165.robot.subsystems.roller.constants.FunnelConstants;
import com.team1165.robot.subsystems.roller.io.RollerIO;
import com.team1165.robot.subsystems.roller.io.RollerIO.RollerIOInputs;
import com.team1165.robot.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputs inputs = new RollerIOInputs();
  private final FunnelState currentState = FunnelState.IDLE;

  private final LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Funnel/Speeds/Intake", FunnelConstants.Voltages.intake);
  private final LoggedTunableNumber manualForwardVoltage = new LoggedTunableNumber("Funnel/Speeds/ManualForward", FunnelConstants.Voltages.manualForward);
  private final LoggedTunableNumber manualReverseVoltage = new LoggedTunableNumber("Funnel/Speeds/ManualReverse", FunnelConstants.Voltages.manualReverse);

  public enum FunnelState {
    IDLE,
    INTAKE,
    MANUAL_FORWARD,
    MANUAL_REVERSE,
    CUSTOM_MANUAL
  }

  public Funnel(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLE:
        io.stop();
        break;
      case INTAKE:
        io.runVolts(5.0);
    }
  }
}
