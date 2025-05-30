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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Class containing various utilities to interface with SPARK motor controllers. */
public final class SparkUtil {
  /** Stores whether any error was detected by any utility methods, since the last reset. */
  private static boolean sparkStickyFault = false;

  private SparkUtil() {} // Prevent instantiation

  /**
   * Creates and configures a SPARK ({@link SparkBase}) motor controller with the provided
   * configuration.
   *
   * @param config The configuration of the SPARK motor controller.
   * @return The new {@link SparkBase} created.
   */
  public static SparkBase createNewSpark(SparkConfig config) {
    // Create the SPARK based on the model provided
    var spark =
        config.model() == SparkModel.SparkFlex
            ? new SparkFlex(config.canId(), config.motorType())
            : new SparkMax(config.canId(), config.motorType());

    // Configure the SPARK with the configuration given
    REVLibError status =
        spark.configure(
            config.configuration(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Alert if the configuration was never successful
    if (status != REVLibError.kOk) {
      new Alert(
              "Hardware Init",
              "SPARK \""
                  + config.name()
                  + "\" (ID: "
                  + config.canId()
                  + ") configuration has failed. Unexpected behavior may occur.",
              AlertType.kWarning)
          .set(true);
    }

    return spark;
  }

  /**
   * Gets the sticky fault status. The sticky fault status represents if any methods called in
   * SparkUtil since the last reset have returned an error.
   *
   * @return The sticky fault status.
   */
  public static boolean getStickyFault() {
    return sparkStickyFault;
  }

  /**
   * Resets the sticky fault status back to false. This should be called right before grabbing
   * information from a SPARK, and called before grabbing information from any other SPARK in a loop
   * period.
   */
  public static void resetStickyFault() {
    sparkStickyFault = false;
  }

  /**
   * Processes a value from a SPARK only if the value is valid (does not error).
   *
   * @param spark The SPARK that the value is being grabbed from.
   * @param supplier The value to try to get from the SPARK.
   * @param consumer The consumer that will process the value if valid.
   */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /**
   * Processes a value from a SPARK only if the value is valid (does not error).
   *
   * @param spark The SPARK that the value is being grabbed from.
   * @param supplier The value to try to get from the SPARK.
   * @param consumer The consumer that will process the value if valid.
   */
  public static <T> void ifOk(SparkBase spark, Supplier<T> supplier, Consumer<T> consumer) {
    var value = supplier.get();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /**
   * Returns a value from a SPARK, or the default if the value is invalid.
   *
   * @param spark The SPARK that the value is being grabbed from.
   * @param supplier The value to try to get from the SPARK.
   * @param defaultValue The default value to be returned if the value is invalid.
   * @return The value from the SPARK, or the default if the value is invalid.
   */
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

  /**
   * Returns a value from a SPARK, or the default if the value is invalid.
   *
   * @param spark The SPARK that the value is being grabbed from.
   * @param supplier The value to try to get from the SPARK.
   * @param defaultValue The default value to be returned if the value is invalid.
   * @return The value from the SPARK, or the default if the value is invalid.
   */
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

  /**
   * Returns a value from a SPARK, or the default if the value is invalid.
   *
   * @param spark The SPARK that the value is being grabbed from.
   * @param supplier The value to try to get from the SPARK.
   * @param defaultValue The default value to be returned if the value is invalid.
   * @return The value from the SPARK, or the default if the value is invalid.
   */
  public static <T> T ifOkOrDefault(SparkBase spark, Supplier<T> supplier, T defaultValue) {
    var value = supplier.get();
    if (spark.getLastError() == REVLibError.kOk) {
      return value;
    } else {
      sparkStickyFault = true;
      return defaultValue;
    }
  }
}
