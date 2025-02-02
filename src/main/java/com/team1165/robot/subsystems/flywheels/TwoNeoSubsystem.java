/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.flywheels;

import com.revrobotics.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// THIS IS TEMPORARY IN PLACE OF THE ACTUAL SUBSYSTEM.

public class TwoNeoSubsystem extends SubsystemBase {

  private final SparkMax neo1;
  private final SparkMax neo2;

  /** Creates a new TwoNeoSubsystem. */
  public TwoNeoSubsystem(int neo1ID, int neo2ID) { // Pass in CAN IDs
    neo1 = new SparkMax(neo1ID, MotorType.kBrushless);
    neo2 = new SparkMax(neo2ID, MotorType.kBrushless);

    //  Initialize or configure the NEOs here if needed.  For example:
    // neo1.setSmartCurrentLimit(someLimit);
    // neo2.setOpenLoopRampRate(someRampRate);
  }

  public void runNeo1(double speed) {
    neo1.set(speed);
  }

  public void runNeo2(double speed) {
    neo2.set(speed);
  }

  public void runBothNeos(double speed1, double speed2) {
    neo1.set(speed1);
    neo2.set(speed2);
  }

  public void stopNeo1() {
    neo1.set(0);
  }

  public void stopNeo2() {
    neo2.set(0);
  }

  public void stopBothNeos() {
    neo1.set(0);
    neo2.set(0);
  }

  @Override
  public void periodic() {
    // Telemetry for Neo 1
    SmartDashboard.putNumber("Neo 1 Speed", neo1.get()); // Current speed
    SmartDashboard.putNumber("Neo 1 Current", neo1.getOutputCurrent()); // Current draw
    SmartDashboard.putNumber("Neo 1 Temperature", neo1.getMotorTemperature()); // Temperature
    SmartDashboard.putNumber("Neo 1 Voltage", neo1.getBusVoltage()); // Input Voltage

    // Telemetry for Neo 2
    SmartDashboard.putNumber("Neo 2 Speed", neo2.get());
    SmartDashboard.putNumber("Neo 2 Current", neo2.getOutputCurrent());
    SmartDashboard.putNumber("Neo 2 Temperature", neo2.getMotorTemperature());
    SmartDashboard.putNumber("Neo 2 Voltage", neo2.getBusVoltage());
  }
}
