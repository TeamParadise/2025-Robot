/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.globalconstants;

/**
 * Class containing constants for the CAN bus(ses) of the robot, mainly consisting of the IDs for
 * all the different mechanisms.
 */
public class CANConstants {
  public static final class IDs {
    public static final class RIO {
      public static final int flywheelPrimary = 1;
      public static final int flywheelSecondary = 2;
      public static final int funnelPrimary = 3;
      public static final int funnelSecondary = 4;
    }
  }
}
