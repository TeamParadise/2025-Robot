/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team1165.robot.FieldConstants;
import java.util.ArrayList;

public class ChoreoUtils {
  /**
   * Method to flip a trajectory across the X axis, mirroring to the other side of the current
   * alliance.
   */
  public static Trajectory<SwerveSample> flipOverX(Trajectory<SwerveSample> traj, String newName) {
    // Create an array to store the flipped swerve states
    var flippedStates = new ArrayList<SwerveSample>();

    // Flip each swerve state
    for (var state : traj.samples()) {
      flippedStates.add(
          new SwerveSample(
              state.t,
              state.x,
              FieldConstants.fieldWidthMeters - state.y,
              -state.heading,
              state.vx,
              -state.vy,
              -state.omega,
              state.ax,
              -state.ay,
              -state.alpha,
              new double[] {
                state.moduleForcesX()[1],
                state.moduleForcesX()[0],
                state.moduleForcesX()[3],
                state.moduleForcesX()[2]
              },
              new double[] {
                -state.moduleForcesY()[1],
                -state.moduleForcesY()[0],
                -state.moduleForcesY()[3],
                -state.moduleForcesY()[2]
              }));
    }

    // Return the newly created trajectory
    return new Trajectory<>(newName, flippedStates, traj.splits(), traj.events());
  }
}
