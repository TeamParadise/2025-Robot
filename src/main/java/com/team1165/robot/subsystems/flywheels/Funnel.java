/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.flywheels;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {

  private final SparkMax neo1;
  private final SparkMax neo2;

  /** Creates a new TwoNeoSubsystem. */
  public Funnel(int neo1ID, int neo2ID) { // Pass in CAN IDs
    neo1 = new SparkMax(neo1ID, MotorType.kBrushless);
    neo2 = new SparkMax(neo2ID, MotorType.kBrushless);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(neo1, true); // Make neo2 follow neo1
    followerConfig.smartCurrentLimit(40);
    followerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    neo2.configure(
        followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.smartCurrentLimit(40);
    leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    neo1.configure(
        followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runBothNeos(double speed1, double speed2) {
    neo1.set(speed1);
  }

  public void stopBothNeos() {
    neo1.set(0);
  }

  // instead of time I would like a boolean function that checks if if its in the shooter.
  public SequentialCommandGroup runNeosFor3Seconds(double speed) {
    Timer timer = new Timer();
    return new SequentialCommandGroup(
        new RunCommand(
                () -> {
                  neo1.set(speed);
                },
                this)
            .beforeStarting(() -> timer.start()),
        new RunCommand(() -> {}, this)
            .until(() -> timer.get() >= 3.0)
            .finallyDo(
                (interrupted) -> {
                  neo1.set(0);
                  timer.stop();
                  timer.reset();
                }));
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
