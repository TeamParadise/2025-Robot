// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.flywheels;

import com.team1165.robot.subsystems.flywheels.io.FlywheelsIO;
import com.team1165.robot.subsystems.flywheels.io.FlywheelsIOInputsAutoLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);
  }

  public Command percentCommand(DoubleSupplier percent) {
    return startEnd(() -> runPercent(percent.getAsDouble()), this::stop)
        .withName("Flywheels Basic Percent");
  }

  public Command percentCommands(DoubleSupplier leftPercent, DoubleSupplier rightPercent) {
    return startEnd(
            () -> runPercent(leftPercent.getAsDouble(), rightPercent.getAsDouble()), this::stop)
        .withName("Flywheels Basic Double Percent");
  }

  public void runPercent(double percent) {
    io.runPercent(percent, percent);
  }

  public void runPercent(double leftPercent, double rightPercent) {
    io.runPercent(leftPercent, rightPercent);
  }

  public void stop() {
    io.stop();
  }

  public boolean isDrawingHighCurrent() {
    return Math.abs(inputs.leftSupplyCurrentAmps) > 15.0;
  }
}
