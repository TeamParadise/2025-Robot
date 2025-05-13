/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.logging;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.team1165.robot.util.vendor.rev.SparkUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;

/**
 * Class that provides easy logging of all the values needed from a motor (controller) to log
 * through AdvantageKit IO classes.
 */
public class MotorData {
  /** The applied voltage to the motor controller. */
  public double appliedVolts = 0.0;

  /** Whether the motor controller is connected. Might be unreliable with a SPARK. */
  public boolean connected = false;

  /** Whether a fault is currently active on the motor controller. */
  public boolean faultActive = false;

  /** String of the currently active faults on the motor controller. */
  public String faults = "";

  /** The temperature of the motor. */
  public double motorTemperatureCelsius = 0.0;

  /**
   * The output current of the motor (controller). This is the general output current on a SPARK and
   * the torque current on a Talon.
   */
  public double outputCurrentAmps = 0.0;

  /**
   * The position reported by the motor controller. Typically motor rotations, but could be
   * different depending on the specific conversion factors on the motor controller.
   */
  public double position = 0.0;

  /** The temperature of the motor controller. Not applicable on a SPARK. */
  public double processorTemperatureCelsius = 0.0;

  /** The supply current provided to the motor controller. */
  public double supplyCurrentAmps = 0.0;

  /**
   * The velocity reported by the motor controller. Typically reported in motor rotations per
   * minute, but could be different depending on the specific conversion factors on the motor
   * controller.
   */
  public double velocity = 0.0;

  /**
   * Method to log this {@link MotorData} to a {@link LogTable} from an AdvantageKit IO class. Call
   * this inside your IO's {@code toLog()}. Make sure the {@code key} matches the one you call
   * {@code fromLog()} with!
   *
   * @param table The table to log to.
   * @param key The key value to log everything under ("Primary" will log everything under the
   *     "Primary/*" key).
   */
  public void toLog(LogTable table, String key) {
    table.put(key + "/AppliedVolts", appliedVolts);
    table.put(key + "/Connected", connected);
    table.put(key + "/FaultActive", faultActive);
    table.put(key + "/Faults", faults);
    table.put(key + "/MotorTemperatureCelsius", motorTemperatureCelsius);
    table.put(key + "/OutputCurrentAmps", outputCurrentAmps);
    table.put(key + "/Position", position);
    table.put(key + "/ProcessorTemperatureCelsius", processorTemperatureCelsius);
    table.put(key + "/SupplyCurrentAmps", supplyCurrentAmps);
    table.put(key + "/Velocity", velocity);
  }

  /**
   * Method to grab the values from a log of this {@link MotorData} from a {@link LogTable} of an
   * AdvantageKit IO class. Call this inside your IO's {@code fromLog()}. Make sure the {@code key}
   * matches the one you call {@code toLog()} with!
   *
   * @param table The table to grab logs from.
   * @param key The key value to grab the logs from ("Primary" will grab everything under the
   *     "Primary/*" key).
   */
  public void fromLog(LogTable table, String key) {
    appliedVolts = table.get(key + "/AppliedVolts", appliedVolts);
    connected = table.get(key + "/Connected", connected);
    faultActive = table.get(key + "/FaultActive", faultActive);
    faults = table.get(key + "/Faults", faults);
    motorTemperatureCelsius = table.get(key + "/MotorTemperatureCelsius", motorTemperatureCelsius);
    outputCurrentAmps = table.get(key + "/OutputCurrentAmps", outputCurrentAmps);
    position = table.get(key + "/Position", position);
    processorTemperatureCelsius =
        table.get(key + "/ProcessorTemperatureCelsius", processorTemperatureCelsius);
    supplyCurrentAmps = table.get(key + "/SupplyCurrentAmps", supplyCurrentAmps);
    velocity = table.get(key + "/Velocity", velocity);
  }

  /** {@link MotorData} class that uses a REV SPARK motor controller with encoder to log data. */
  public static class SparkMotorData extends MotorData {
    // Motor controller and encoder to grab data from
    private final SparkBase spark;
    private final RelativeEncoder encoder;

    /**
     * Debouncer in order to ignore any skipped CAN frames instead of reporting that the controller
     * is disconnected.
     */
    private final Debouncer connectedDebouncer = new Debouncer(0.25);

    /**
     * Constructs a {@link SparkMotorData} using the specified constants.
     *
     * @param spark The {@link SparkBase} to log data from.
     */
    public SparkMotorData(SparkBase spark) {
      this.spark = spark;
      this.encoder = spark.getEncoder();
    }

    /**
     * Updates the motor data using the values from the REV SPARK motor controller linked with this
     * instance.
     */
    public void update() {
      // Reset the sticky fault value
      SparkUtil.resetStickyFault();

      // Get applied output since it's used multiple times later on
      double appliedOutput = spark.getAppliedOutput();

      appliedVolts =
          SparkUtil.ifOkOrDefault(spark, () -> spark.getBusVoltage() * appliedOutput, appliedVolts);
      faultActive = SparkUtil.ifOkOrDefault(spark, spark::hasActiveFault, faultActive);
      if (faultActive) {
        Faults sparkFaults = spark.getFaults();
        faults =
            (sparkFaults.other ? "other " : "")
                + (sparkFaults.motorType ? "motorType " : "")
                + (sparkFaults.sensor ? "sensor " : "")
                + (sparkFaults.can ? "can " : "")
                + (sparkFaults.temperature ? "temperature " : "")
                + (sparkFaults.gateDriver ? "gateDriver " : "")
                + (sparkFaults.escEeprom ? "escEeprom " : "")
                + (sparkFaults.firmware ? "firmware " : "");
      } else {
        faults = "";
      }
      motorTemperatureCelsius =
          SparkUtil.ifOkOrDefault(spark, spark::getMotorTemperature, motorTemperatureCelsius);
      outputCurrentAmps =
          SparkUtil.ifOkOrDefault(spark, spark::getOutputCurrent, outputCurrentAmps);
      position = SparkUtil.ifOkOrDefault(spark, encoder::getPosition, position);
      processorTemperatureCelsius = 0.0; // Not compatible with SPARKs
      supplyCurrentAmps = outputCurrentAmps * appliedOutput; // Approximate, not exact
      velocity = SparkUtil.ifOkOrDefault(spark, encoder::getVelocity, velocity);

      // After updating everything, check if SparkUtil reports any connection issues/sticky fault
      connected = connectedDebouncer.calculate(!SparkUtil.getStickyFault());
    }
  }

  public static class TalonMotorData extends MotorData {
    private final TalonFX talon;

    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Integer> faultFieldSignal;
    private final List<StatusSignal<Boolean>> faultsSignals =
        new ArrayList<StatusSignal<Boolean>>(99);
    private final StatusSignal<Temperature> motorTemperatureSignal;
    private final StatusSignal<Current> outputCurrentSignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Temperature> processorTemperatureSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;

    public TalonMotorData(TalonFX talon) {
      this.talon = talon;

      appliedVoltsSignal = talon.getMotorVoltage();
      faultFieldSignal = talon.getFaultField();
      faultsSignals.add(talon.getFault_BootDuringEnable());
      faultsSignals.add(talon.getFault_DeviceTemp());
      faultsSignals.add(talon.getFault_Hardware());
      faultsSignals.add(talon.getFault_ProcTemp());
      motorTemperatureSignal = talon.getDeviceTemp();
      outputCurrentSignal = talon.getTorqueCurrent();
      positionSignal = talon.getPosition();
      processorTemperatureSignal = talon.getProcessorTemp();
      supplyCurrentSignal = talon.getSupplyCurrent();
      velocitySignal = talon.getVelocity();
    }
  }
}
