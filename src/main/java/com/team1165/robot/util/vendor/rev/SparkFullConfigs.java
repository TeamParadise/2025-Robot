/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.rev;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Class to store record-based configuration for SPARK motor controllers, encompassing everything
 * needed to initialize and configure the motor controller.
 */
public class SparkFullConfigs {
  /**
   * Class to store a record-based configuration for a SPARK motor controller, encompassing
   * everything needed to initialize and configure the motor controller.
   */
  public abstract static class SparkFullConfig {
    /** The model of the SPARK (MAX or FLEX). */
    public final SparkModel model;

    /** The CAN ID of the SPARK. */
    public final int canId;

    /** The type of motor (brushless/brushed) connected to the SPARK. */
    public final MotorType motorType;

    /** The configuration to apply to the SPARK. */
    public final SparkBaseConfig config;

    /**
     * Constructs a {@link SparkFullConfig} using the specified constants.
     *
     * @param model The model of the SPARK (MAX or FLEX).
     * @param canId The CAN ID of the SPARK.
     * @param motorType The type of motor (brushless/brushed) connected to the SPARK.
     * @param config The configuration to apply to the SPARK.
     */
    protected SparkFullConfig(
        SparkModel model, int canId, MotorType motorType, SparkBaseConfig config) {
      this.model = model;
      this.canId = canId;
      this.motorType = motorType;
      this.config = config;
    }
  }

  /**
   * Class to store a record-based configuration for a SPARK MAX motor controller, encompassing
   * everything need to initialize and configure the motor controller.
   */
  public static class SparkMaxFullConfig extends SparkFullConfig {
    /**
     * Constructs a {@link SparkMaxFullConfig} using the specified constants.
     *
     * @param canId The CAN ID of the SPARK MAX.
     * @param motorType The type of motor (brushless/brushed) connected to the SPARK MAX.
     * @param config The configuration to apply to the SPARK MAX.
     */
    public SparkMaxFullConfig(int canId, MotorType motorType, SparkMaxConfig config) {
      super(SparkModel.SparkMax, canId, motorType, config);
    }
  }

  /**
   * Class to store a record-based configuration for a SPARK FLEX motor controller, encompassing
   * everything need to initialize and configure the motor controller.
   */
  public static class SparkFlexFullConfig extends SparkFullConfig {
    /**
     * Constructs a {@link SparkFlexFullConfig} using the specified constants.
     *
     * @param canId The CAN ID of the SPARK FLEX.
     * @param motorType The type of motor (brushless/brushed) connected to the SPARK FLEX.
     * @param config The configuration to apply to the SPARK FLEX.
     */
    public SparkFlexFullConfig(int canId, MotorType motorType, SparkMaxConfig config) {
      super(SparkModel.SparkFlex, canId, motorType, config);
    }
  }
}
