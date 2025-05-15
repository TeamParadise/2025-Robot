/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.ctre;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * Class to store record-based configurations for CTR Electronics Phoenix 6 devices. This includes
 * devices like Talon FX(S), CANcoders, Pigeon 2.0s, and more.
 */
public final class PhoenixDeviceConfigs {
  private PhoenixDeviceConfigs() {} // Prevent instantiation

  /**
   * Record-based configuration for a CANcoder, encompassing everything needed to initialize and
   * configure the CANcoder.
   *
   * @param name The name that the CANcoder should be called in logs and alerts. This, ideally,
   *     should match the name given on the CAN bus.
   * @param canId The CAN ID of the CANcoder.
   * @param canBus The CAN bus that the CANcoder is located on ("rio" for the roboRIO CAN bus,
   *     otherwise, the name of the CANivore bus).
   * @param config The configuration to apply to the CANcoder.
   */
  public record CANcoderConfig(
      String name, int canId, String canBus, CANcoderConfiguration config) {}

  /**
   * Record-based configuration for a CANrange, encompassing everything needed to initialize and
   * configure the CANrange.
   *
   * @param name The name that the CANrange should be called in logs and alerts. This, ideally,
   *     should match the name given on the CAN bus.
   * @param canId The CAN ID of the CANrange.
   * @param canBus The CAN bus that the CANrange is located on ("rio" for the roboRIO CAN bus,
   *     otherwise, the name of the CANivore bus).
   * @param config The configuration to apply to the CANrange.
   */
  public record CANrangeConfig(
      String name, int canId, String canBus, CANrangeConfiguration config) {}

  /**
   * Record-based configuration for a Pigeon 2.0, encompassing everything needed to initialize and
   * configure the Pigeon.
   *
   * @param name The name that the Pigeon should be called in logs and alerts. This, ideally, should
   *     match the name given on the CAN bus.
   * @param canId The CAN ID of the Pigeon.
   * @param canBus The CAN bus that the Pigeon is located on ("rio" for the roboRIO CAN bus,
   *     otherwise, the name of the CANivore bus).
   * @param config The configuration to apply to the Pigeon.
   */
  public record PigeonConfig(String name, int canId, String canBus, Pigeon2Configuration config) {}

  /**
   * Record-based configuration for a Talon FX, encompassing everything needed to initialize and
   * configure the Talon FX.
   *
   * @param name The name that the Talon FX should be called in logs and alerts. This, ideally,
   *     should match the name given on the CAN bus.
   * @param canId The CAN ID of the Talon FX.
   * @param canBus The CAN bus that the Talon FX is located on ("rio" for the roboRIO CAN bus,
   *     otherwise, the name of the CANivore bus).
   * @param config The configuration to apply to the Talon FX.
   */
  public record TalonFXConfig(String name, int canId, String canBus, TalonFXConfiguration config) {}
}
