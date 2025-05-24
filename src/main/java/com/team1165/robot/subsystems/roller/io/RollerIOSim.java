/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.team1165.robot.util.logging.MotorData.GenericMotorData;
import edu.wpi.first.units.measure.Current;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/**
 * A hardware interface/implementation layer for a basic wheel/roller subsystem powered by two
 * simulated motors. These two motors are usually controlled together, but they can be controlled
 * separately if needed.
 */
public class RollerIOSim implements RollerIO {
  private final MapleMotorSim primarySim;
  private final MapleMotorSim secondarySim;
  private final GenericMotorController primaryController;
  private final GenericMotorController secondaryController;

  // "Motor" data to log
  private final GenericMotorData primaryMotorData = new GenericMotorData();
  private final GenericMotorData secondaryMotorData = new GenericMotorData();

  public RollerIOSim(SimMotorConfigs config, Current currentLimit) {
    primarySim = new MapleMotorSim(config);
    secondarySim = new MapleMotorSim(config);
    primaryController = primarySim.useSimpleDCMotorController().withCurrentLimit(currentLimit);
    secondaryController = secondarySim.useSimpleDCMotorController().withCurrentLimit(currentLimit);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    primarySim.update(Seconds.of(0.020));
    secondarySim.update(Seconds.of(0.020));
    // Update the motor data
    primaryMotorData.update(
        primarySim.getAppliedVoltage().in(Volts),
        primarySim.getStatorCurrent().in(Amps),
        primarySim.getAngularPosition().in(Rotations),
        primarySim.getSupplyCurrent().in(Amps),
        primarySim.getVelocity().in(Rotations.per(Minute)));

    secondaryMotorData.update(
        secondarySim.getAppliedVoltage().in(Volts),
        secondarySim.getStatorCurrent().in(Amps),
        secondarySim.getAngularPosition().in(Rotations),
        secondarySim.getSupplyCurrent().in(Amps),
        secondarySim.getVelocity().in(Rotations.per(Minute)));

    inputs.primaryMotor = primaryMotorData;
    inputs.secondaryMotor = secondaryMotorData;
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
    primaryController.requestVoltage(Volts.of(primaryVoltage));
    secondaryController.requestVoltage(Volts.of(secondaryVoltage));
  }

  @Override
  public void stop() {
    primaryController.requestVoltage(Volts.of(0));
    secondaryController.requestVoltage(Volts.of(0));
  }
}
