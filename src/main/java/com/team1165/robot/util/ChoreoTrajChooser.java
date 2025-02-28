/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util;

import choreo.Choreo;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Simple class that provides the ability to choose between all available trajectories in the
 * "deploy" folder for Choreo.
 */
public class ChoreoTrajChooser {
  // Create loggable dashboard chooser
  private final LoggedDashboardChooser<AutoTrajectory> chooser;

  // Create auto routine variable
  private final AutoRoutine autoRoutine;

  /**
   * Creates a new ChoreoTrajChooser, for handling a chooser input sent via NetworkTables.
   *
   * @param autoRoutine The AutoRoutine to use for this trajectory chooser, used for creating
   *     commands.
   * @param key The key for the chooser, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/SmartDashboard/{key}" when logged.
   */
  public ChoreoTrajChooser(AutoRoutine autoRoutine, String key) {
    // Initialize the chooser
    chooser = new LoggedDashboardChooser<AutoTrajectory>(key);

    // Initialize the auto routine
    this.autoRoutine = autoRoutine;

    // Add a "None" value to the chooser, and make it the default
    chooser.addDefaultOption("None", null);

    // Add all the Choreo trajectories to the chooser
    for (String traj : Choreo.availableTrajectories()) {
      chooser.addOption(traj, autoRoutine.trajectory(traj));
    }
  }

  public Command get(boolean resetOdometry) {
    var trajectory = chooser.get();

    if (trajectory != null) {
      autoRoutine
          .active()
          .onTrue(
              resetOdometry
                  ? Commands.sequence(trajectory.resetOdometry(), trajectory.cmd())
                  : trajectory.cmd());
    } else {
      autoRoutine.active().onTrue(Commands.none());
    }

    return autoRoutine.cmd();
  }
}
