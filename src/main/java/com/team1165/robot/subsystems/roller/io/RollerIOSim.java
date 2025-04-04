/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A hardware interface/implementation layer for a basic wheel/roller subsystem powered by two
 * simulated motors. These two motors are usually controlled
 * together, but they can be controlled separately if needed.
 */
public class RollerIOSim implements RollerIO {
  private final DCMotor gearbox;
  private final DCMotorSim primarySim;
  private final DCMotorSim secondarySim;
  private double primaryAppliedVoltage;
  private double secondaryAppliedVoltage;

  private RollerIOSim(DCMotor motor, double moi) {
    gearbox = motor;
    primarySim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 1, moi), motor);
    secondarySim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 1, moi), motor);
  }

  /**
   * Run the motors together at a specific voltage.
   *
   * @param voltage The voltage to run the rollers at.
   */
  @Override
  public void runVolts(double voltage) {
    runVolts(voltage, voltage);
  }

  /**
   * Run the motors separately at different voltages. This should only be used if the motors are not
   * physically coupled by any means.
   *
   * @param primaryVoltage The voltage to run the primary motor at.
   * @param secondaryVoltage The voltage to run the secondary motor at.
   */
  @Override
  public void runVolts(double primaryVoltage, double secondaryVoltage) {
    primaryAppliedVoltage = MathUtil.clamp(primaryVoltage, -12.0, 12.0);
    primarySim.setInputVoltage(primaryVoltage);
    secondarySim.setInputVoltage(secondaryVoltage);
  }
}
