/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveConstants {
  /** Constants for autonomous driving and line up of the robot. */
  public static final class PathConstants {
    public static final PIDConstants translation = new PIDConstants(10, 0, 0);
    public static final PIDConstants rotation = new PIDConstants(7, 0, 0);
    public static final PPHolonomicDriveController ppDriveController =
        new PPHolonomicDriveController(translation, rotation);
  }

  /** How often the simulation thread should be run. */
  public static final double simulationLoopPeriod = 0.005;
}
