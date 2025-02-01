// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.flywheels;

import com.team1165.robot.Constants;
import com.team1165.robot.Constants.Mode;

public class FlywheelConstants {
  public static Mode robotMode = Mode.REAL;
  public static final FlywheelConfig flywheelConfig =
      switch (Constants.getRobot()) {
        case COMPBOT -> new FlywheelConfig(4, 0, 2000.0);
        case SIMBOT -> new FlywheelConfig(0, 0, 5000.0);
      };

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case COMPBOT -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
      };

  public record FlywheelConfig(int leftID, int rightID, double maxAcclerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
