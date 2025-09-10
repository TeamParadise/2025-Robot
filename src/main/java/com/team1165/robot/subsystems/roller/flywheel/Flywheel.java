/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel;

import com.team1165.robot.subsystems.roller.flywheel.sensor.SensorIO;
import com.team1165.robot.subsystems.roller.flywheel.sensor.SensorIO.SensorIOInputs;
import com.team1165.robot.subsystems.roller.io.RollerIO;
import com.team1165.robot.subsystems.roller.io.RollerIO.RollerIOInputs;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import com.team1165.robot.util.statemachine.OverridableStateMachine;
import com.team1165.robot.util.statemachine.StateUtils;
import java.util.EnumMap;
import org.littletonrobotics.junction.Logger;

/** State-machine-based Flywheel subsystem, powered by two motors. */
public class Flywheel extends OverridableStateMachine<FlywheelState> {
  // RollerIO objects
  private final RollerIO io;
  private final RollerIOInputs inputs = new RollerIOInputs();

  // SensorIO objects
  private final SensorIO sensorIO;
  private final SensorIOInputs sensorInputs = new SensorIOInputs();

  private final EnumMap<FlywheelState, LoggedTunableNumber> tunableMap =
      StateUtils.createTunableNumberMap(name + "/Voltages", FlywheelState.class);

  public Flywheel(RollerIO io, SensorIO sensorIO) {
    super(FlywheelState.IDLE);
    this.io = io;
    this.sensorIO = sensorIO;
  }

  public double getOutputCurrent() {
    // TODO: Maybe get the average of the values?
    return inputs.primaryMotor.outputCurrentAmps;
  }

  public double getSensorDistance() {
    return sensorInputs.distance;
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    sensorIO.updateInputs(sensorInputs);
    Logger.processInputs(name, inputs);
    Logger.processInputs(name + "/Sensor", inputs);
  }

  @Override
  protected void transition() {
    io.runVolts(tunableMap.get(getCurrentState()).get());
  }
}
