/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.Conduit;

import java.nio.ByteBuffer;

/** ConduitJni */
class ConduitJni {

  static {
    System.loadLibrary("wpilibio");
  }

  public static native ByteBuffer getBuffer();

  public static native void capture();

  public static native void start();

  public static native void configurePowerDistribution(int moduleID, int moduleType);
}
