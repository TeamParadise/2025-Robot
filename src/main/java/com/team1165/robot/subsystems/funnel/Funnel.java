// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.funnel;

import com.team1165.robot.subsystems.funnel.io.FunnelIO;
import com.team1165.robot.subsystems.funnel.io.FunnelIOInputsAutoLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  public Funnel(FunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);
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

  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.leftSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.rightSupplyCurrentAmps) > 50.0;
  }
}
