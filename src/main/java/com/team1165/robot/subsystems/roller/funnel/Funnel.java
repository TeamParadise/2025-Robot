/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel;

import com.team1165.robot.util.statemachine.StateMachine;
import com.team1165.robot.subsystems.roller.funnel.constants.FunnelConstants;
import com.team1165.robot.subsystems.roller.io.RollerIO;
import com.team1165.robot.subsystems.roller.io.RollerIO.RollerIOInputs;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

/** State-machine based Funnel subsystem, powered by two motors. */
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
  private final LoggedTunableNumber customManualVoltage =
      new LoggedTunableNumber("Funnel/Speeds/CustomManual", 0.0);

  public Funnel(RollerIO io) {
    super(FunnelState.IDLE);
    this.io = io;
    transition(FunnelState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);
  }

  @Override
  public void setState(FunnelState state) {
    super.setState(state);
  }

  public Command stateCommand(FunnelState state) {
    return Commands.run(() -> setState(state), this);
  }

  @Override
  protected void updateInputs() {
    io.updateInputs(inputs);
  }

  @Override
  protected void transition(FunnelState goalState) {
    switch (goalState) {
      case IDLE:
        io.runVolts(0.0);
        break;
      case INTAKE:
        io.runVolts(intakeVoltage.get());
        break;
      case MANUAL_FORWARD:
        io.runVolts(manualForwardVoltage.get());
        break;
      case MANUAL_REVERSE:
        io.runVolts(manualReverseVoltage.get());
        break;
      case CUSTOM_MANUAL:
        io.runVolts(customManualVoltage.get());
    }
  }
}
