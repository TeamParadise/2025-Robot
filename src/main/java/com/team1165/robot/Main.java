/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import com.team1165.robot.globalconstants.Constants;
import com.team1165.robot.globalconstants.Constants.Mode;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization method. Do not perform any initialization here.
   *
   * <p>If you change your main Robot class (name), change the parameter type.
   */
  public static void main(String... args) {
    // Set the robot mode based on current running conditions
    Constants.robotMode =
        RobotBase.isReal()
            ? Mode.REAL
            : (Arrays.asList(args).contains("--replay") ? Mode.REPLAY : Mode.SIM);
    // Start the main Robot class
    RobotBase.startRobot(Robot::new);
  }
}
