/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.Pivot;

import com.team1165.robot.Constants;
import com.team1165.robot.Constants.Mode;


public class PivotConstants {
  public static Mode robotMode = Mode.REAL;
  public static final PivotConstants.PivotConfig pivotConfig =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PivotConstants.PivotConfig(12,  2000.0);
        case SIMBOT -> new PivotConstants.PivotConfig(10,  5000.0);
      };

  public static final PivotConstants.Gains gains =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PivotConstants.Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
        case SIMBOT -> new PivotConstants.Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
      };

  public record PivotConfig(int theID, double maxAcclerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
