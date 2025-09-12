/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel.sensor;

import com.ctre.phoenix6.hardware.CANrange;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.CANrangeConfig;
import com.team1165.robot.util.vendor.ctre.PhoenixUtil;

/** A hardware interface/implementation layer for a CTRE CANrange sensor. */
public class SensorIOCTRE implements SensorIO {
  private final CANrange sensor;

  public SensorIOCTRE(CANrangeConfig config) {
    // Initialize CANrange
    sensor = PhoenixUtil.createNewCANrange(config);

    // Add values to Phoenix refresh
    PhoenixUtil.registerSignals(config.canBus(), sensor.getDistance());
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = sensor.isConnected();
    inputs.distance = sensor.getDistance(false).getValueAsDouble();
  }
}
