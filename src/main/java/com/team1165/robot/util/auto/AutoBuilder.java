/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1165.robot.OdysseusManager;
import com.team1165.robot.globalconstants.FieldConstants.CoralStationLocation;
import com.team1165.robot.globalconstants.FieldConstants.Reef;
import com.team1165.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * Class to publish values to the dashboard for autonomous configuration, and to build the final
 * auto as soon as the autonomous period starts.
 */
public class AutoBuilder {
  private static AutoBuilder instance;
  private static AutoRoutine currentAutoRoutine;

  private static final LoggedNetworkString reef1 =
      new LoggedNetworkString("Auto/Score/FirstLocation", "J");
  private static final LoggedNetworkString reef2 =
      new LoggedNetworkString("Auto/Score/SecondLocation", "K");
  private static final LoggedNetworkString reef3 =
      new LoggedNetworkString("Auto/Score/ThirdLocation", "L");
  private static final LoggedNetworkString reef4 =
      new LoggedNetworkString("Auto/Score/FourthLocation", "A");
  private static final LoggedDashboardChooser<Reef.Level> level1 =
      new LoggedDashboardChooser<>("Auto/Score/FirstLevel", new SendableChooser<>());
  private static final LoggedDashboardChooser<Reef.Level> level2 =
      new LoggedDashboardChooser<>("Auto/Score/SecondLevel", new SendableChooser<>());
  private static final LoggedDashboardChooser<Reef.Level> level3 =
      new LoggedDashboardChooser<>("Auto/Score/ThirdLevel", new SendableChooser<>());
  private static final LoggedDashboardChooser<Reef.Level> level4 =
      new LoggedDashboardChooser<>("Auto/Score/FourthLevel", new SendableChooser<>());
  private static final LoggedDashboardChooser<CoralStationLocation> cs1 =
      new LoggedDashboardChooser<>("Auto/CS/First", new SendableChooser<>());
  private static final LoggedDashboardChooser<CoralStationLocation> cs2 =
      new LoggedDashboardChooser<>("Auto/CS/Second", new SendableChooser<>());
  private static final LoggedDashboardChooser<CoralStationLocation> cs3 =
      new LoggedDashboardChooser<>("Auto/CS/Third", new SendableChooser<>());
  private static final LoggedDashboardChooser<CoralStationLocation> cs4 =
      new LoggedDashboardChooser<>("Auto/CS/Fourth", new SendableChooser<>());
  private static final LoggedNetworkBoolean justLeave =
      new LoggedNetworkBoolean("Auto/JustLeave", false);
  private static final LoggedNetworkBoolean pushPartner =
      new LoggedNetworkBoolean("Auto/PushPartner", false);
  private static final LoggedNetworkNumber sequencesToRun =
      new LoggedNetworkNumber("Auto/SequencesToRun", 4);

  private AutoBuilder() {
    addLevelOptions(level1);
    addLevelOptions(level2);
    addLevelOptions(level3);
    addLevelOptions(level4);
    addCSOptions(cs1);
    addCSOptions(cs2);
    addCSOptions(cs3);
    addCSOptions(cs4);
  }

  private static void addLevelOptions(LoggedDashboardChooser<Reef.Level> levelChooser) {
    levelChooser.addOption("L1", Reef.Level.L1);
    levelChooser.addOption("L2", Reef.Level.L2);
    levelChooser.addOption("L3", Reef.Level.L3);
    levelChooser.addDefaultOption("L4", Reef.Level.L4);
  }

  private static void addCSOptions(LoggedDashboardChooser<CoralStationLocation> csChooser) {
    csChooser.addDefaultOption("LCS", CoralStationLocation.LCS);
    csChooser.addOption("RCS", CoralStationLocation.RCS);
  }

  public Command buildAutoCommand(OdysseusManager robot, Drive drive) {
    if (justLeave.get()) {
      return drive
          .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(1.5))
          .withTimeout(0.75);
    }

    var numberOfSegments = (int) Math.round(sequencesToRun.get());
    var segments = new AutoSegmentConfig[numberOfSegments];
    for (int i = 0; i < numberOfSegments; i++) {
      if (i == 0) {
        segments[i] =
            new AutoSegmentConfig(Reef.Location.valueOf(reef1.get()), level1.get(), cs1.get());
      } else if (i == 1) {
        segments[i] =
            new AutoSegmentConfig(Reef.Location.valueOf(reef2.get()), level2.get(), cs2.get());
      } else if (i == 2) {
        segments[i] =
            new AutoSegmentConfig(Reef.Location.valueOf(reef3.get()), level3.get(), cs3.get());
      } else if (i == 3) {
        segments[i] =
            new AutoSegmentConfig(Reef.Location.valueOf(reef4.get()), level4.get(), cs4.get());
      }
    }

    return new AutoRoutine(robot, drive, pushPartner.get(), segments).getAutoCommand();
  }

  public static AutoBuilder getInstance() {
    if (instance == null) {
      instance = new AutoBuilder();
    }

    return instance;
  }
}
