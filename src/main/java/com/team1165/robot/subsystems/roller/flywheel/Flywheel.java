/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel;

import com.team1165.robot.subsystems.roller.flywheel.sensor.DetectionMode;
import com.team1165.robot.subsystems.roller.flywheel.sensor.SensorIO;
import com.team1165.robot.subsystems.roller.flywheel.sensor.SensorIO.SensorIOInputs;
import com.team1165.robot.subsystems.roller.flywheel.sensor.SensorIOInputsAutoLogged;
import com.team1165.robot.subsystems.roller.io.RollerIO;
import com.team1165.robot.subsystems.roller.io.RollerIO.RollerIOInputs;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import com.team1165.robot.util.statemachine.OverridableStateMachine;
import com.team1165.robot.util.statemachine.StateUtils;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.EnumMap;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** State-machine-based Flywheel subsystem, powered by two motors. */
public class Flywheel extends OverridableStateMachine<FlywheelState> {
  // RollerIO objects
  private final RollerIO io;
  private final RollerIOInputs inputs = new RollerIOInputs();

  // Sensor objects
  private final SensorIO sensorIO;
  private final SensorIOInputsAutoLogged sensorInputs = new SensorIOInputsAutoLogged();
  private final LoggedTunableNumber sensorDistanceThreshold =
      new LoggedTunableNumber(name + "/Sensor/DistanceThreshold", 1.0);
  private DetectionMode detectionMode = FlywheelConstants.defaultDetectionMode;
  private final Alert sensorAlert = new Alert("Scoring mechanism sensor disconnected!", AlertType.kError);

  private final EnumMap<FlywheelState, LoggedTunableNumber> tunableMap =
      StateUtils.createTunableNumberMap(name + "/Voltages", FlywheelState.class);

  public Flywheel(RollerIO io, SensorIO sensorIO) {
    super(FlywheelState.IDLE);
    this.io = io;
    this.sensorIO = sensorIO;
  }

  @AutoLogOutput(key = "Flywheel/DetectionMode")
  public DetectionMode getDetectionMode() {
    return detectionMode;
  }

  public boolean getCurrentHold(double current) {
    return Math.abs(inputs.primaryMotor.outputCurrentAmps) > current;
  }

  @AutoLogOutput(key = "Flywheel/Sensors/Hold")
  public boolean getSensorHold() {
    // Default to true if the distance sensor is not active
    return detectionMode != DetectionMode.DISTANCE_SENSOR || sensorInputs.distance < sensorDistanceThreshold.get();
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    sensorIO.updateInputs(sensorInputs);
    Logger.processInputs(name, inputs);
    Logger.processInputs(name + "/Sensor", sensorInputs);

    // Detect if the CANrange has become disconnected and fall back to current
    if (FlywheelConstants.defaultDetectionMode != DetectionMode.CURRENT) {
      detectionMode = sensorInputs.connected ? DetectionMode.DISTANCE_SENSOR : DetectionMode.CURRENT;
      sensorAlert.set(!sensorInputs.connected);
    }
  }

  @Override
  protected void transition() {
    io.runVolts(tunableMap.get(getCurrentState()).get());
  }
}
