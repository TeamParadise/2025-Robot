/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.ctre;

import com.ctre.phoenix6.hardware.CANcoder;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.CANcoderConfig;

public final class PhoenixUtil {
  private PhoenixUtil() {} // Prevent instantiation

  public static CANcoder createNewCANcoder(String name, CANcoderConfig config) {
    // Create the CANcoder with the configuration values
    var cancoder = new CANcoder(config.canId(), config.canBus());


  }
}
