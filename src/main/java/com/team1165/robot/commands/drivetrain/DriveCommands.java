/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.drivetrain;

import com.team1165.robot.util.logging.LoggedTunableNumber;

public class DriveCommands {
  private static final LoggedTunableNumber deadband = new LoggedTunableNumber("Commands/Drive/Teleop/Deadband", 0.1);
}
