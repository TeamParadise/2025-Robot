/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor;

public record TalonFXFaults(
    boolean anyFaultActive,
    boolean temperature,
    boolean overCurrent,
    boolean sensor,
    boolean stall,
    boolean gateDriver,
    boolean other) {}
