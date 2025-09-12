/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag;

import com.team1165.robot.util.statemachine.State;

/** All the possible states for the {@link ATVision} subsystem. */
public enum ATVisionState implements State {
  /** Single-tag vision without the trig-based gyro combination. Default PhotonVision method, more accurate up-close, less accurate when far away. */
  SINGLE_TAG_3D(),
  /** Single-tag vision with the trig-based gyro combination. More accurate further away, less accurate close up. */
  SINGLE_TAG_TRIG()
}
