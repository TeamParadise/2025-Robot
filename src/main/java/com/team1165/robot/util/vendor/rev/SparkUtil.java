/*
 * File originally made by: Mechanical Advantage - FRC 6328
 * Copyright (c) 2025 Team 6328 (https://github.com/Mechanical-Advantage)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.vendor.rev;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /** Return a value from a Spark (or the default if the value is invalid). */
  public static double ifOkOrDefault(
      SparkBase spark, DoubleSupplier supplier, double defaultValue) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      return value;
    } else {
      sparkStickyFault = true;
      return defaultValue;
    }
  }

  /** Return a value from a Spark (or the default if the value is invalid). */
  public static boolean ifOkOrDefault(
      SparkBase spark, BooleanSupplier supplier, boolean defaultValue) {
    boolean value = supplier.getAsBoolean();
    if (spark.getLastError() == REVLibError.kOk) {
      return value;
    } else {
      sparkStickyFault = true;
      return defaultValue;
    }
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      }
    }
  }
}
