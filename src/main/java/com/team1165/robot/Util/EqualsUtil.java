/*
 * Copyright (c) 2024-2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util;

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, 1e-9);
  }

  /** Extension methods for wpi geometry objects */
  public static class GeomExtensions {
    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
      return EqualsUtil.epsilonEquals(twist.dx, other.dx)
          && EqualsUtil.epsilonEquals(twist.dy, other.dy)
          && EqualsUtil.epsilonEquals(twist.dtheta, other.dtheta);
    }
  }
}
