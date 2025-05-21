/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1165.robot.FieldConstants.CoralStationLocation;
import com.team1165.robot.FieldConstants.Reef;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.flywheels.Flywheels;
import com.team1165.robot.subsystems.roller.funnel.Funnel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AutoBuilder {
  private static AutoBuilder instance;

  private static final LoggedNetworkString reef1 =
      new LoggedNetworkString("Auto/Score/FirstLocation", "A");
  private static final LoggedNetworkString reef2 =
      new LoggedNetworkString("Auto/Score/SecondLocation", "A");
  private static final LoggedNetworkString reef3 =
      new LoggedNetworkString("Auto/Score/ThirdLocation", "A");
  private static final LoggedDashboardChooser<Reef.Level> level1 =
      new LoggedDashboardChooser<>("Auto/Score/FirstLevel", new SendableChooser<>());
  private static final LoggedDashboardChooser<Reef.Level> level2 =
      new LoggedDashboardChooser<>("Auto/Score/SecondLevel", new SendableChooser<>());
  private static final LoggedDashboardChooser<Reef.Level> level3 =
      new LoggedDashboardChooser<>("Auto/Score/ThirdLevel", new SendableChooser<>());
  private static final LoggedNetworkNumber delayBeforeStart =
      new LoggedNetworkNumber("Auto/StartDelay", 0.0);
  private static final LoggedNetworkBoolean justLeave =
      new LoggedNetworkBoolean("Auto/JustLeave", false);
  private static final LoggedNetworkBoolean pushPartner =
      new LoggedNetworkBoolean("Auto/PushPartner", false);
  private static final LoggedNetworkNumber scoreDelay1 =
      new LoggedNetworkNumber("Auto/Score/FirstDelay", 0.0);
  private static final LoggedNetworkNumber scoreDelay2 =
      new LoggedNetworkNumber("Auto/Score/SecondDelay", 0.0);
  private static final LoggedNetworkNumber scoreDelay3 =
      new LoggedNetworkNumber("Auto/Score/ThirdDelay", 0.0);
  private static final LoggedDashboardChooser<CoralStationLocation> cs1 =
      new LoggedDashboardChooser<>("Auto/CS/First", new SendableChooser<>());
  private static final LoggedDashboardChooser<CoralStationLocation> cs2 =
      new LoggedDashboardChooser<>("Auto/CS/Second", new SendableChooser<>());
  private static final LoggedDashboardChooser<CoralStationLocation> cs3 =
      new LoggedDashboardChooser<>("Auto/CS/Third", new SendableChooser<>());
  private static final LoggedNetworkNumber csDelay1 =
      new LoggedNetworkNumber("Auto/CS/FirstDelay", 0.5);
  private static final LoggedNetworkNumber csDelay2 =
      new LoggedNetworkNumber("Auto/CS/SecondDelay", 0.5);
  private static final LoggedNetworkNumber csDelay3 =
      new LoggedNetworkNumber("Auto/CS/ThirdDelay", 0.5);
  private static final LoggedNetworkNumber sequencesToRun =
      new LoggedNetworkNumber("Auto/SequencesToRun", 3);

  private AutoBuilder() {
    addLevelOptions(level1);
    addLevelOptions(level2);
    addLevelOptions(level3);
    addCSOptions(cs1);
    addCSOptions(cs2);
    addCSOptions(cs3);
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

  public Command buildAutoCommand(
      Drive drive, Elevator elevator, Flywheels flywheels, Funnel funnel) {
    if (justLeave.get()) {
      return drive
          .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-1.5))
          .withTimeout(0.75);
    }

    var numberOfSegments = (int) Math.round(sequencesToRun.get());
    var segments = new AutoSegmentConfig[numberOfSegments];
    for (int i = 0; i < numberOfSegments; i++) {
      if (i == 0) {
        segments[i] =
            new AutoSegmentConfig(
                Reef.Location.valueOf(reef1.get()),
                level1.get(),
                Seconds.of(scoreDelay1.get()),
                cs1.get(),
                Seconds.of(csDelay1.get()));
      } else if (i == 1) {
        segments[i] =
            new AutoSegmentConfig(
                Reef.Location.valueOf(reef2.get()),
                level2.get(),
                Seconds.of(scoreDelay2.get()),
                cs2.get(),
                Seconds.of(csDelay2.get()));
      } else if (i == 2) {
        segments[i] =
            new AutoSegmentConfig(
                Reef.Location.valueOf(reef3.get()),
                level3.get(),
                Seconds.of(scoreDelay3.get()),
                cs3.get(),
                Seconds.of(csDelay3.get()));
      }
    }

    var autoRoutine =
        new AutoRoutine(Seconds.of(delayBeforeStart.get()), pushPartner.get(), segments);
    return autoRoutine.getAutoCommand(drive, elevator, flywheels, funnel);
  }

  public static AutoBuilder getInstance() {
    if (instance == null) {
      instance = new AutoBuilder();
    }

    return instance;
  }
}
