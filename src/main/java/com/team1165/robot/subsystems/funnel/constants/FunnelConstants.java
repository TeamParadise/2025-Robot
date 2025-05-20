// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.funnel.constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FunnelConstants {
  public static final int leftID = 3;
  public static final int rightID = 4;

  public static final SparkBaseConfig motorConfig =
      new SparkMaxConfig().smartCurrentLimit(50).idleMode(IdleMode.kBrake);

  public static final class Speeds {
    public static final double intake = 0.4;
  }
}
