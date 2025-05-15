/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.CANcoderConfig;
import java.util.Arrays;
import java.util.function.Supplier;

public final class PhoenixUtil {
  // Signals for synchronized refresh across devices
  private static BaseStatusSignal[] signalArrayA = new BaseStatusSignal[0];
  private static BaseStatusSignal[] signalArrayB = new BaseStatusSignal[0];

  private PhoenixUtil() {} // Prevent instantiation

  public static CANcoder createNewCANcoder(CANcoderConfig config, StatusSignal<?>... statusSignals) {
    // Create the CANcoder with the configuration values
    var cancoder = new CANcoder(config.canId(), config.canBus());
  }

  /**
   * Attempts to run the method/supplier until no error is produced.
   *
   * @param maxAttempts The maximum number of times to try before giving up.
   * @param method The method to run and check if it was successful.
   * @return If the method was ever successful.
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> method) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = method.get();
      if (error.isOK()) return true;
    }
    return false;
  }

  public static void registerSignals(StatusSignal<?>... signals) {
    // TODO: Make sure this code works.
    var signalListA = Arrays.asList(signalArrayA);
    var signalListB = Arrays.asList(signalArrayB);

    for (int i = 0; i < signals.length; i++) {
      if (signalListA.isEmpty()) {
        signalListA.add(signals[i]);
      } else {

      }
    }
  }

  /** Refreshes all signals across devices registered with this class. */
  public static void refreshAll() {
    if (signalArrayA.length > 0) BaseStatusSignal.refreshAll(signalArrayA);
    if (signalArrayB.length > 0) BaseStatusSignal.refreshAll(signalArrayB);
  }
}
