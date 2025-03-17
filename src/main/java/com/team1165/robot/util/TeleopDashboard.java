/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util;

import com.team1165.robot.FieldConstants.Reef.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class TeleopDashboard {
  private static TeleopDashboard instance;
  private static final LoggedNetworkString reef =
      new LoggedNetworkString("Teleop/Teleop Reef", "A");
  private static final LoggedDashboardChooser<Level> level =
      new LoggedDashboardChooser<Level>("Teleop/Teleop Level", new SendableChooser<Level>());

  private TeleopDashboard() {
    level.addDefaultOption("L1", Level.L1);
    level.addOption("L2", Level.L2);
    level.addOption("L3", Level.L3);
    level.addOption("L4", Level.L4);
  }

  public Location getReefLocation() {
    return Location.valueOf(reef.get());
  }

  public Level getLevel() {
    return level.get();
  }

  public static TeleopDashboard getInstance() {
    if (instance == null) {
      instance = new TeleopDashboard();
    }

    return instance;
  }
}
