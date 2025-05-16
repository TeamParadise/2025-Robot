/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.CANBusJNI;
import com.team1165.robot.util.constants.CANFrequency;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.CANcoderConfig;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.CANrangeConfig;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.PigeonConfig;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.TalonFXConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.function.Supplier;

/** Class containing various utilities to interface with CTR Electronics Phoenix 6 devices. */
public final class PhoenixUtil {
  // Signals for synchronized refresh across devices
  private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];
  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  private PhoenixUtil() {} // Prevent instantiation

  /**
   * Creates and configures a CTRE CANcoder ({@link CANcoder}) with the provided config.
   *
   * @param config The full config of the CANcoder.
   * @return The new {@link CANcoder} created.
   */
  public static CANcoder createNewCANcoder(CANcoderConfig config) {
    // Create the CANcoder with the configuration values
    var cancoder = new CANcoder(config.canId(), config.canBus());

    // Configure the CANcoder with the configuration given
    boolean failed = tryUntilOk(5, () -> cancoder.getConfigurator().apply(config.configuration()));

    // Alert if the configuration was never successful
    if (failed) {
      new Alert(
              "CANcoder \""
                  + config.name()
                  + "\" (ID: "
                  + config.canId()
                  + ") configuration has failed. Unexpected behavior may occur.",
              AlertType.kWarning)
          .set(true);
    }

    // Explicitly enable required status signals for remote sensors
    BaseStatusSignal.setUpdateFrequencyForAll(
        CANFrequency.FAST.getFrequency(CANBusJNI.JNI_IsNetworkFD(config.canBus())),
        cancoder.getAbsolutePosition(),
        cancoder.getPosition(),
        cancoder.getVelocity(),
        cancoder.getMagnetHealth());

    // Disable unused status signals (can always be enabled later)
    cancoder.optimizeBusUtilization(0, 0.1);

    return cancoder;
  }

  /**
   * Creates and configures a CTRE CANrange ({@link CANrange}) with the provided config.
   *
   * @param config The full config of the CANrange.
   * @return The new {@link CANrange} created.
   */
  public static CANrange createNewCANrange(CANrangeConfig config) {
    // Create the CANrange with the configuration values
    var canrange = new CANrange(config.canId(), config.canBus());

    // Configure the CANrange with the configuration given
    boolean failed = tryUntilOk(5, () -> canrange.getConfigurator().apply(config.configuration()));

    // Alert if the configuration was never successful
    if (failed) {
      new Alert(
              "CANrange \""
                  + config.name()
                  + "\" (ID: "
                  + config.canId()
                  + ") configuration has failed. Unexpected behavior may occur.",
              AlertType.kWarning)
          .set(true);
    }

    // Explicitly enable required status signals for remote sensors
    BaseStatusSignal.setUpdateFrequencyForAll(
        CANFrequency.FAST.getFrequency(CANBusJNI.JNI_IsNetworkFD(config.canBus())),
        canrange.getIsDetected());

    // Disable unused status signals (can always be enabled later)
    canrange.optimizeBusUtilization(0, 0.1);

    return canrange;
  }

  /**
   * Creates and configures a CTRE Pigeon 2.0 ({@link Pigeon2}) with the provided config.
   *
   * @param config The full config of the Pigeon.
   * @return The new {@link Pigeon2} created.
   */
  public static Pigeon2 createNewPigeon(PigeonConfig config) {
    // Create the Pigeon with the configuration values
    var pigeon = new Pigeon2(config.canId(), config.canBus());

    // Configure the Pigeon with the configuration given
    boolean failed = tryUntilOk(5, () -> pigeon.getConfigurator().apply(config.configuration()));

    // Alert if the configuration was never successful
    if (failed) {
      new Alert(
              "Pigeon \""
                  + config.name()
                  + "\" (ID: "
                  + config.canId()
                  + ") configuration has failed. Unexpected behavior may occur.",
              AlertType.kWarning)
          .set(true);
    }

    // Explicitly enable required status signals for remote sensors
    BaseStatusSignal.setUpdateFrequencyForAll(
        CANFrequency.FAST.getFrequency(CANBusJNI.JNI_IsNetworkFD(config.canBus())),
        pigeon.getYaw(),
        pigeon.getPitch(),
        pigeon.getRoll());

    // Disable unused status signals (can always be enabled later)
    pigeon.optimizeBusUtilization(0, 0.1);

    return pigeon;
  }

  /**
   * Creates and configures a Talon FX ({@link TalonFX}) with the provided config.
   *
   * @param config The full config of the Talon FX.
   * @return The new {@link TalonFX} created.
   */
  public static TalonFX createNewTalonFX(TalonFXConfig config) {
    // Create the Talon FX with the configuration values
    var talon = new TalonFX(config.canId(), config.canBus());

    // Configure the CANrange with the configuration given
    boolean failed = tryUntilOk(5, () -> talon.getConfigurator().apply(config.configuration()));

    // Alert if the configuration was never successful
    if (failed) {
      new Alert(
          "Talon FX \""
              + config.name()
              + "\" (ID: "
              + config.canId()
              + ") configuration has failed. Unexpected behavior may occur.",
          AlertType.kWarning)
          .set(true);
    }

    // Explicitly enable required status signals for sensors and motors that are being followed
    BaseStatusSignal.setUpdateFrequencyForAll(
        CANFrequency.FAST.getFrequency(CANBusJNI.JNI_IsNetworkFD(config.canBus())),
        talon.getDutyCycle(),
        talon.getMotorVoltage(),
        talon.getTorqueCurrent(),
        talon.getPosition(),
        talon.getVelocity());

    // Disable unused status signals (can always be enabled later)
    talon.optimizeBusUtilization(0, 0.1);

    return talon;
  }

  /** Refreshes all signals registered with this class. */
  public static void refreshAll() {
    if (canivoreSignals.length > 0) BaseStatusSignal.refreshAll(canivoreSignals);
    if (rioSignals.length > 0) BaseStatusSignal.refreshAll(rioSignals);
  }

  /**
   * Registers a set of signals for synchronized refresh through {@link #refreshAll()}. Through a
   * single call of this method, all the signals registered must be on a single CAN bus. To add
   * signals from the secondary CAN bus, call this method a second time with that CAN bus passed in,
   * and the signals on it.
   *
   * @param canBus The CAN bus that the signals are located on ("rio" or "" for the roboRIO CAN bus,
   *     otherwise, the name of the CANivore bus).
   * @param signals The status signals to be registered with this class.
   */
  public static void registerSignals(String canBus, BaseStatusSignal... signals) {
    if (CANBusJNI.JNI_IsNetworkFD(canBus)) {
      // Create a new CANivore signals array with the new signals added
      BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
      System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
      canivoreSignals = newSignals;
    } else {
      // Create a new RIO signals array with the new signals added
      BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
      System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
      rioSignals = newSignals;
    }
  }

  /**
   * Attempts to run the method/supplier until no error is produced.
   *
   * @param maxAttempts The maximum number of times to try before giving up.
   * @param method The method to run and check if it was successful.
   * @return If the method was never successful.
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> method) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = method.get();
      if (error.isOK()) return false;
    }
    return true;
  }

}
