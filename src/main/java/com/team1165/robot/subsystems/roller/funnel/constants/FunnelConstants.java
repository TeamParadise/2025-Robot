/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team1165.robot.util.vendor.rev.SparkConfig;

public class FunnelConstants {
  private static final SparkBaseConfig baseMotorConfig =
      new SparkMaxConfig().smartCurrentLimit(50).idleMode(IdleMode.kBrake);
  public static final SparkConfig primaryMotorConfig =
      SparkConfig.sparkMax("FunnelPrimary", 3, MotorType.kBrushless, baseMotorConfig);
  public static final SparkConfig secondaryMotorConfig =
      SparkConfig.sparkMax("FunnelSecondary", 4, MotorType.kBrushless, baseMotorConfig);

  public static final class Voltages {
    public static final double intake = 5.0;
    public static final double manualForward = 3.0;
    public static final double manualReverse = -3.0;
  }
}
