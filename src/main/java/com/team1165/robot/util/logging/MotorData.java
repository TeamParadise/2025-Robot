/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.logging;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.team1165.robot.util.constants.CANFrequency;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.TalonFXConfig;
import com.team1165.robot.util.vendor.ctre.PhoenixUtil;
import com.team1165.robot.util.vendor.rev.SparkConfig;
import com.team1165.robot.util.vendor.rev.SparkUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.LogTable;

/**
 * Class that provides easy logging of all the default values from a motor (controller) through
 * AdvantageKit IO input classes.
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
   * Method to log this {@link MotorData} to a {@link LogTable} from an AdvantageKit input class.
   *
   * <p>Call this inside your input's {@code toLog()}. Make sure the {@code key} matches the one you
   * call {@code fromLog()} with!
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
   * AdvantageKit input class.
   *
   * <p>Call this inside your input's {@code fromLog()}. Make sure the {@code key} matches the one
   * you call {@code toLog()} with!
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

  /**
   * Generic {@link MotorData} class used for any controllers that don't already exist in {@link
   * MotorData}, and for simulation.
   *
   * <p>Values are manually passed in each time the {@link #update} method is called.
   */
  public static class GenericMotorData extends MotorData {
    /**
     * Updates the motor data using the values passed into this method. If any values do not exist
     * (or for simulation, are not simulated), and are not required for a specific subsystem, pass
     * in <code>0.0</code>.
     *
     * @param appliedVolts The value to set {@link #appliedVolts} to.
     * @param connected The value to set {@link #connected} to.
     * @param faultActive The value to set {@link #faultActive} to.
     * @param faults The value to set {@link #faults} to.
     * @param motorTemperatureCelsius The value to set {@link #motorTemperatureCelsius} to.
     * @param outputCurrentAmps The value to set {@link #outputCurrentAmps} to.
     * @param position The value to set {@link #position} to.
     * @param processorTemperatureCelsius The value to set {@link #processorTemperatureCelsius} to.
     * @param supplyCurrentAmps The value to set {@link #supplyCurrentAmps} to.
     * @param velocity The value to set {@link #velocity} to.
     */
    public void update(
        double appliedVolts,
        boolean connected,
        boolean faultActive,
        String faults,
        double motorTemperatureCelsius,
        double outputCurrentAmps,
        double position,
        double processorTemperatureCelsius,
        double supplyCurrentAmps,
        double velocity) {
      this.appliedVolts = appliedVolts;
      this.connected = connected;
      this.faultActive = faultActive;
      this.faults = faults;
      this.motorTemperatureCelsius = motorTemperatureCelsius;
      this.outputCurrentAmps = outputCurrentAmps;
      this.position = position;
      this.processorTemperatureCelsius = processorTemperatureCelsius;
      this.supplyCurrentAmps = supplyCurrentAmps;
      this.velocity = velocity;
    }

    /**
     * Updates the motor data using the values passed into this method. If any values do not exist
     * (or for simulation, are not simulated), and are not required for a specific subsystem, pass
     * in <code>0.0</code>.
     *
     * @param appliedVolts The value to set {@link #appliedVolts} to.
     * @param outputCurrentAmps The value to set {@link #outputCurrentAmps} to.
     * @param position The value to set {@link #position} to.
     * @param supplyCurrentAmps The value to set {@link #supplyCurrentAmps} to.
     * @param velocity The value to set {@link #velocity} to.
     */
    public void update(
        double appliedVolts,
        double outputCurrentAmps,
        double position,
        double supplyCurrentAmps,
        double velocity) {
      update(
          appliedVolts,
          true,
          false,
          "",
          0.0,
          outputCurrentAmps,
          position,
          0.0,
          supplyCurrentAmps,
          velocity);
    }
  }

  /**
   * {@link MotorData} class that uses a REV SPARK (MAX/FLEX) motor controller with a relative
   * encoder to log data.
   */
  public static class SparkMotorData extends MotorData {
    // SPARK and encoder to grab data from
    private final SparkBase spark;
    private final RelativeEncoder encoder;

    // Alerts to send if any issues arise with the SPARK or motor
    private final Alert connectedAlert;
    private final Alert faultAlert;

    /**
     * Debouncer to ensure that a few bad CAN frames won't cause the SPARK to report as
     * disconnected.
     */
    private final Debouncer connectedDebouncer = new Debouncer(0.20);

    /**
     * Constructs a {@link SparkMotorData} using the specified constants.
     *
     * @param spark The {@link SparkBase} to log data from.
     * @param config The {@link SparkConfig} for this SPARK. Only used for name and CAN ID.
     */
    public SparkMotorData(SparkBase spark, SparkConfig config) {
      // Get the SPARK and encoder to log data from
      this.spark = spark;
      this.encoder = spark.getEncoder();

      // Create alerts with the name and CAN ID of the SPARK
      connectedAlert =
          new Alert(
              "Hardware",
              "SPARK \"" + config.name() + "\" (ID: " + config.canId() + ") is disconnected!",
              AlertType.kError);
      faultAlert =
          new Alert(
              "Hardware",
              "SPARK \"" + config.name() + "\" (ID: " + config.canId() + ") has active faults!",
              AlertType.kError);
    }

    /**
     * Updates the motor data using the values from the REV SPARK (MAX/FLEX) motor controller linked
     * with this instance.
     */
    public void update() {
      // Reset the sticky fault value from SparkUtil
      SparkUtil.resetStickyFault();

      // Check if there are any active faults, if there are, activate an alert and save the faults
      faultAlert.set(
          faultActive = SparkUtil.ifOkOrDefault(spark, spark::hasActiveFault, faultActive));
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

      // Get applied output since it's used multiple times later on
      double appliedOutput = spark.getAppliedOutput();

      // Get values from the SPARK and save them
      appliedVolts =
          SparkUtil.ifOkOrDefault(spark, () -> spark.getBusVoltage() * appliedOutput, appliedVolts);
      motorTemperatureCelsius =
          SparkUtil.ifOkOrDefault(spark, spark::getMotorTemperature, motorTemperatureCelsius);
      outputCurrentAmps =
          SparkUtil.ifOkOrDefault(spark, spark::getOutputCurrent, outputCurrentAmps);
      position = SparkUtil.ifOkOrDefault(spark, encoder::getPosition, position);
      processorTemperatureCelsius = 0.0; // Not compatible with SPARKs
      supplyCurrentAmps = outputCurrentAmps * appliedOutput; // Approximate, not exact
      velocity = SparkUtil.ifOkOrDefault(spark, encoder::getVelocity, velocity);

      // After updating everything, check if SparkUtil reports any connection issues/sticky fault
      connectedAlert.set(!(connected = connectedDebouncer.calculate(!SparkUtil.getStickyFault())));
    }
  }

  /**
   * {@link MotorData} class that uses status signals from a Talon FX motor controller to log data.
   */
  public static class TalonFXMotorData extends MotorData {
    // Status signals providing the data to log
    private final StatusSignal<Voltage> appliedVoltsSignal;
    private final StatusSignal<Integer> faultFieldSignal;
    private final StatusSignal<Boolean> bootDuringEnableFaultSignal;
    private final StatusSignal<Boolean> deviceTempFaultSignal;
    private final StatusSignal<Boolean> hardwareFaultSignal;
    private final StatusSignal<Boolean> procTempFaultSignal;
    private final StatusSignal<Temperature> motorTemperatureSignal;
    private final StatusSignal<Current> outputCurrentSignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Temperature> processorTemperatureSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;

    // Alerts to send if any issues arise with the Talon FX or motor
    private final Alert connectedAlert;
    private final Alert faultAlert;

    /**
     * Debouncer to ensure that a few bad CAN frames won't cause the Talon FX to report as
     * disconnected.
     */
    private final Debouncer connectedDebouncer = new Debouncer(0.20);

    /**
     * Constructs a {@link TalonFXMotorData} using the specified constants.
     *
     * @param talon The {@link TalonFX} to log data from.
     * @param config The {@link TalonFXConfig} for this Talon FX. Only used for name, CAN ID, and
     *     CAN bus.
     */
    public TalonFXMotorData(TalonFX talon, TalonFXConfig config) {
      // Get status signals from the Talon FX
      appliedVoltsSignal = talon.getMotorVoltage();
      faultFieldSignal = talon.getFaultField();
      bootDuringEnableFaultSignal = talon.getFault_BootDuringEnable();
      deviceTempFaultSignal = talon.getFault_DeviceTemp();
      hardwareFaultSignal = talon.getFault_Hardware();
      procTempFaultSignal = talon.getFault_ProcTemp();
      motorTemperatureSignal = talon.getDeviceTemp();
      outputCurrentSignal = talon.getTorqueCurrent();
      positionSignal = talon.getPosition();
      processorTemperatureSignal = talon.getProcessorTemp();
      supplyCurrentSignal = talon.getSupplyCurrent();
      velocitySignal = talon.getVelocity();

      // Set the update frequency and register the signals
      PhoenixUtil.setFrequencyAndRegister(
          config.canBus(),
          CANFrequency.MEDIUM,
          appliedVoltsSignal,
          motorTemperatureSignal,
          outputCurrentSignal,
          positionSignal,
          processorTemperatureSignal,
          supplyCurrentSignal,
          velocitySignal);
      PhoenixUtil.setFrequencyAndRegister(
          config.canBus(),
          CANFrequency.SLOW,
          faultFieldSignal,
          bootDuringEnableFaultSignal,
          deviceTempFaultSignal,
          hardwareFaultSignal,
          procTempFaultSignal);

      // Create alerts with the name and ID of the Talon FX
      connectedAlert =
          new Alert(
              "Hardware",
              "Talon FX \"" + config.name() + "\" (ID: " + config.canId() + ") is disconnected!",
              AlertType.kError);
      faultAlert =
          new Alert(
              "Hardware",
              "Talon FX \"" + config.name() + "\" (ID: " + config.canId() + ") has active faults!",
              AlertType.kError);
    }

    /**
     * Updates the motor data using the status signals from the Talon FX motor controller linked
     * with this instance.
     */
    public void update() {
      // Check if there are any active faults, if there are, activate an alert and save the faults
      faultAlert.set(faultActive = faultFieldSignal.getValue() != 0);
      if (faultActive) {
        faults =
            (bootDuringEnableFaultSignal.getValue() ? "BootDuringEnable " : "")
                + (deviceTempFaultSignal.getValue() ? "DeviceTemp " : "")
                + (hardwareFaultSignal.getValue() ? "Hardware " : "")
                + (procTempFaultSignal.getValue() ? "ProcTemp " : "");
      } else {
        faults = "";
      }

      // Get values from the status signals and save them
      appliedVolts = appliedVoltsSignal.getValueAsDouble();
      motorTemperatureCelsius = motorTemperatureSignal.getValueAsDouble();
      outputCurrentAmps = outputCurrentSignal.getValueAsDouble();
      position = positionSignal.getValueAsDouble();
      processorTemperatureCelsius = processorTemperatureSignal.getValueAsDouble();
      supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
      velocity = velocitySignal.getValueAsDouble();

      // After updating everything, check if there are any reported connection issues
      connectedAlert.set(
          !(connected =
              connectedDebouncer.calculate(
                  BaseStatusSignal.isAllGood(
                      motorTemperatureSignal, processorTemperatureSignal, supplyCurrentSignal))));
    }
  }
}
