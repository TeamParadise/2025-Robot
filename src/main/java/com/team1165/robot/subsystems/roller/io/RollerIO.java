/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.io;

import com.team1165.robot.util.logging.MotorData;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A hardware interface/implementation layer for a basic wheel/roller subsystem powered by two
 * motors. These two motors are usually controlled together, but they can be controlled separately
 * if needed.
 */
public interface RollerIO {
  /** Class used to store the IO values of a basic roller subsystem. */
  class RollerIOInputs implements LoggableInputs, Cloneable {
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
    public RollerIOInputs clone() {
      RollerIOInputs copy = new RollerIOInputs();
      copy.primaryMotor = this.primaryMotor;
      copy.secondaryMotor = this.secondaryMotor;
      return copy;
    }
  }

  /**
   * Updates a {@link RollerIOInputs} instance with the latest updates from this {@link RollerIO}.
   *
   * @param inputs A {@link RollerIOInputs} instance to update.
   */
  default void updateInputs(RollerIOInputs inputs) {}

  /**
   * Run the motors together at a specific voltage.
   *
   * @param voltage The voltage to run the rollers at.
   */
  default void runVolts(double voltage) {}

  /**
   * Run the motors separately at different voltages. This should only be used if the motors are not
   * physically coupled by any means.
   *
   * @param primaryVoltage The voltage to run the primary motor at.
   * @param secondaryVoltage The voltage to run the secondary motor at.
   */
  default void runVolts(double primaryVoltage, double secondaryVoltage) {}

  /** Stops both of the motors (sets the output to zero). */
  default void stop() {}

  /**
   * Enables or disables brake mode on both of the roller motors.
   *
   * @param enabled Whether to enable brake mode.
   */
  default void setBrakeMode(boolean enabled) {}
}
