/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.rev;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * Class to store a record-based config for a SPARK motor controller, encompassing everything needed
 * to initialize and configure the motor controller.
 *
 * @param name The name that the SPARK should be called in logs and alerts.
 * @param model The model of the SPARK (MAX or FLEX).
 * @param canId The CAN ID of the SPARK.
 * @param motorType The type of motor (brushless/brushed) connected to the SPARK.
 * @param configuration The configuration to apply to the SPARK.
 */
public record SparkConfig(
    String name, SparkModel model, int canId, MotorType motorType, SparkBaseConfig configuration) {
  /**
   * Constructs a {@link SparkConfig} for a SPARK MAX using the specified constants.
   *
   * @param name The name that the SPARK should be called in logs and alerts.
   * @param canId The CAN ID of the SPARK MAX.
   * @param motorType The type of motor (brushless/brushed) connected to the SPARK MAX.
   * @param configuration The configuration to apply to the SPARK MAX.
   */
  public static SparkConfig sparkMax(
      String name, int canId, MotorType motorType, SparkBaseConfig configuration) {
    return new SparkConfig(name, SparkModel.SparkMax, canId, motorType, configuration);
  }

  /**
   * Constructs a {@link SparkConfig} for a SPARK FLEX using the specified constants.
   *
   * @param name The name that the SPARK should be called in logs and alerts.
   * @param canId The CAN ID of the SPARK FLEX.
   * @param motorType The type of motor (brushless/brushed) connected to the SPARK FLEX.
   * @param configuration The configuration to apply to the SPARK FLEX.
   */
  public static SparkConfig sparkFlex(
      String name, int canId, MotorType motorType, SparkBaseConfig configuration) {
    return new SparkConfig(name, SparkModel.SparkFlex, canId, motorType, configuration);
  }
}
