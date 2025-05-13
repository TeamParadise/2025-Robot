/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team1165.robot.util.vendor.rev.SparkConfig;
import com.team1165.robot.util.vendor.rev.SparkFullConfigs.SparkMaxFullConfig;

public class FunnelConstants {
  private static final SparkMaxConfig baseMotorConfig =
      new SparkMaxConfig().apply(); // figure this out, REVLib sucks.
  private static final SparkConfig primaryMotorConfig =
      SparkConfig.sparkMax(3, MotorType.kBrushless, baseMotorConfig);
  private static final SparkConfig secondaryMotorConfig =
      SparkConfig.sparkMax(4, MotorType.kBrushless, baseMotorConfig);

  public static final class Voltages {
    public static final double intake = 5.0;
    public static final double manualForward = 3.0;
    public static final double manualReverse = -3.0;
  }
}
