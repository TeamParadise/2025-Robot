/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.team1165.robot.util.logging.MotorData;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** A hardware interface/implementation layer for a basic two-motor elevator subsystem. */
public interface ElevatorIO {
  /** Class used to store the IO values of a basic elevator subsystem. */
  class ElevatorIOInputs implements LoggableInputs, Cloneable {
    /**
     * Data from the primary motor of the subsystem. Most of the time, any data needed should be
     * grabbed from here.
     */
    public MotorData primaryMotor = new MotorData();

    /** Data from the secondary motor of the subsystem. */
    public MotorData secondaryMotor = new MotorData();

    @Override
    public void toLog(LogTable table) {
      primaryMotor.toLog(table, "Primary");
      secondaryMotor.toLog(table, "Secondary");
    }

    @Override
    public void fromLog(LogTable table) {
      primaryMotor.fromLog(table, "Primary");
      secondaryMotor.fromLog(table, "Secondary");
    }

    @Override
    public ElevatorIOInputs clone() {
      ElevatorIOInputs copy = new ElevatorIOInputs();
      copy.primaryMotor = this.primaryMotor;
      copy.secondaryMotor = this.secondaryMotor;
      return copy;
    }
  }

  /**
   * Updates a {@link ElevatorIOInputs} instance with the latest updates from this {@link
   * ElevatorIO}.
   *
   * @param inputs A {@link ElevatorIOInputs} instance to update.
   */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Run the motors together at a specific current.
   *
   * @param amps The current to run the motors at.
   */
  default void runCurrent(double amps) {}

  /**
   * Set the motors to run to a specific position using their PID controller.
   *
   * @param position The position to run the motors to, in the same units that the motor reports.
   */
  default void runPosition(double position) {}

  /**
   * Run the motors together at a specific voltage.
   *
   * @param voltage The voltage to run the motors at.
   */
  default void runVolts(double voltage) {}

  /** Stops both of the motors (sets the output to zero). */
  default void stop() {}

  /**
   * Configures the PID values for both of the motors.
   *
   * @param gains The PID gains to apply to the motors.
   */
  default void setPID(Slot0Configs gains) {}

  /**
   * Configures the Motion Profiling settings for both of the motors.
   *
   * @param config The Motion Profiling configuration to apply to the motors.
   */
  default void setMotionProfiling(MotionMagicConfigs config) {}
}
