/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.logging;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.team1165.robot.util.vendor.rev.SparkUtil;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.LogTable;

/**
 * Class that provides easy logging of all the values needed from a motor to log through
 * AdvantageKit IO classes.
 */
public class MotorData {
  public boolean connected = false;
  public boolean faultActive = false;
  public String faults = "";
  public double appliedVolts = 0.0;
  public double outputCurrentAmps = 0.0;
  public double positionRotations = 0.0;
  public double supplyCurrentAmps = 0.0;
  public double temperatureCelsius = 0.0;
  public double velocityRpm = 0.0;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public void toLog(LogTable table, String key) {
    table.put(key + "/AppliedVolts", appliedVolts);
    table.put(key + "/Connected", connected);
    table.put(key + "/FaultActive", faultActive);
    table.put(key + "/Faults", faults);
    table.put(key + "/OutputCurrentAmps", outputCurrentAmps);
    table.put(key + "/PositionRotations", positionRotations);
    table.put(key + "/SupplyCurrentAmps", supplyCurrentAmps);
    table.put(key + "/TemperatureCelsius", temperatureCelsius);
    table.put(key + "/VelocityRPM", velocityRpm);
  }

  public void fromLog(LogTable table, String key) {
    appliedVolts = table.get(key + "/AppliedVolts", appliedVolts);
    connected = table.get(key + "/Connected", connected);
    faultActive = table.get(key + "/FaultActive", faultActive);
    faults = table.get(key + "/Faults", faults);
    outputCurrentAmps = table.get(key + "/OutputCurrentAmps", outputCurrentAmps);
    positionRotations = table.get(key + "/PositionRotations", positionRotations);
    supplyCurrentAmps = table.get(key + "/SupplyCurrentAmps", supplyCurrentAmps);
    temperatureCelsius = table.get(key + "/TemperatureCelsius", temperatureCelsius);
    velocityRpm = table.get(key + "/VelocityRPM", velocityRpm);
  }

  public void updateFromSpark(SparkBase spark, RelativeEncoder encoder) {
    var appliedOutput = spark.getAppliedOutput();

    appliedVolts =
        SparkUtil.ifOkOrDefault(spark, () -> spark.getBusVoltage() * appliedOutput, appliedVolts);
    faultActive = SparkUtil.ifOkOrDefault(spark, spark::hasActiveFault, faultActive);
    if (faultActive) {
      Faults sparkFaults = spark.getFaults();
      faults =
          (sparkFaults.other ? "other" : "")
              + (sparkFaults.motorType ? "motorType" : "")
              + (sparkFaults.sensor ? "sensor" : "")
              + (sparkFaults.can ? "can" : "")
              + (sparkFaults.temperature ? "temperature" : "")
              + (sparkFaults.gateDriver ? "gateDriver" : "")
              + (sparkFaults.escEeprom ? "escEeprom" : "")
              + (sparkFaults.firmware ? "firmware" : "");
    }
    outputCurrentAmps = SparkUtil.ifOkOrDefault(spark, spark::getOutputCurrent, outputCurrentAmps);
    positionRotations = SparkUtil.ifOkOrDefault(spark, encoder::getPosition, positionRotations);
    supplyCurrentAmps = outputCurrentAmps * appliedOutput;
    temperatureCelsius =
        SparkUtil.ifOkOrDefault(spark, spark::getMotorTemperature, temperatureCelsius);
  }
}
