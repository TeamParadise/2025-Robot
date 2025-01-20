/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems;

import com.team1165.robot.Constants;

public class ConstantsElevator {
  public static final ElevatorConfig flywheelConfig =
      switch (Constants.getRobot()) {
        case COMPBOT -> new ElevatorConfig(4, 0, (1.0 / 2.0), 9000.0);
        case DEVBOT -> new ElevatorConfig(5, 4, (1.0 / 2.0), 6000.0);
        case SIMBOT -> new ElevatorConfig(0, 0, (1.0 / 2.0), 9000.0);
      };

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case COMPBOT -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
        case DEVBOT -> new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
      };

  public record ElevatorConfig(
      int leftID, int rightID, double reduction, double maxAcclerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
