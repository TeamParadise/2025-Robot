/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ElevatorConstants {
  public static final String canBus = "canivore";

  // Intake position
  public static final double intakePosition = 1.1;

  // General characteristics
  public static final double gearRatio = 5.0;
  public static final double sprocketRadiusInches = 0.8755;
  public static final double minHeightInches = 0.0;
  public static final double maxHeightInches = 40.872;

  // PID gains
  public static final Slot0Configs gains =
      new Slot0Configs()
          .withKP(10.0)
          .withKI(0.0)
          .withKD(0.0)
          .withKS(0.22)
          .withKG(0.33)
          .withKV(0.0)
          .withKA(0.0);

  public static final class LeftMotorConstants {
    public static final int canID = 14;
  }

  public static final class RightMotorConstants {
    public static final int canID = 15;
  }
}
