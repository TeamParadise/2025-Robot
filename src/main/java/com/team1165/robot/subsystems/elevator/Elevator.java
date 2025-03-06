/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator;

import com.team1165.robot.FieldConstants.Reef;
import com.team1165.robot.subsystems.elevator.constants.ElevatorConstants;
import com.team1165.robot.subsystems.elevator.io.ElevatorIO;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import com.team1165.robot.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Customizable PID values (if in tuning mode)
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/Tuning/Gains/kP", ElevatorConstants.gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/Tuning/Gains/kI", ElevatorConstants.gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/Tuning/Gains/kD", ElevatorConstants.gains.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/Tuning/Gains/kS", ElevatorConstants.gains.kS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/Tuning/Gains/kG", ElevatorConstants.gains.kG);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/Tuning/Gains/kV", ElevatorConstants.gains.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/Tuning/Gains/kA", ElevatorConstants.gains.kA);
  private static final LoggedTunableNumber positionOverride =
      new LoggedTunableNumber("Elevator/Tuning/PositionOverride", 0.0);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Disconnected alerts
  private final Alert leftDisconnected =
      new Alert("Left ElevatorMotor disconnected!", AlertType.kError);
  private final Alert rightDisconnected =
      new Alert("Right ElevatorMotor disconnected!", AlertType.kError);

  private double homePosition = 0.0;
  private boolean emergencyStop = false;
  private boolean brakeMode = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Set alerts
    leftDisconnected.set(!inputs.leftMotorConnected);
    rightDisconnected.set(!inputs.rightMotorConnected);
  }

  public void stop() {
    io.stop();
    if (!emergencyStop) {
      Logger.recordOutput("Elevator/RunningMode", "Stop");
    }
  }

  public void runAtPercent(double percent) {
    if (!emergencyStop) {
      Logger.recordOutput("Elevator/RunningMode", "BasicVoltage");
      io.runVolts(percent * 12.0);
    }
  }

  public void runVolts(double volts) {
    if (!emergencyStop) {
      Logger.recordOutput("Elevator/RunningMode", "BasicVoltage");
      io.runVolts(volts);
    }
  }

  public void runPosition(double positionInches) {
    if (!emergencyStop) {
      Logger.recordOutput("Elevator/Setpoint", positionInches);
      Logger.recordOutput("Elevator/RunningMode", "ManualPosition");
      io.runPosition(positionInches + homePosition);
    }
  }

  public void runPosition(Reef.Level level) {
    if (!emergencyStop) {
      Logger.recordOutput("Elevator/Setpoint", level.getElevatorHeight());
      Logger.recordOutput("Elevator/RunningMode", "Coral " + level.name());
      io.runPosition(level.getElevatorHeight() + homePosition);
    }
  }

  public void runToIntakePosition() {
    if (!emergencyStop) {
      Logger.recordOutput("Elevator/Setpoint", ElevatorConstants.intakePosition);
      Logger.recordOutput("Elevator/RunningMode", "Intake");
      io.runPosition(ElevatorConstants.intakePosition + homePosition);
    }
  }

  public void setBrakeMode(boolean enabled) {
    if (!emergencyStop) {
      brakeMode = enabled;
      io.setBrakeMode(enabled);
    }
  }

  /**
   * Emergency stop the elevator, immediately stop motors and set to brake mode. Run methods will
   * not do anything.
   */
  public void setEmergencyStop(boolean enabled) {
    if (enabled) {
      Logger.recordOutput("Elevator/RunningMode", "EmergencyStop");
      emergencyStop = true;
      io.stop();
      io.setBrakeMode(false);
    } else {
      Logger.recordOutput("Elevator/RunningMode", "Stop");
      emergencyStop = false;
      io.setBrakeMode(brakeMode);
    }
  }

  public void setHomePosition(double homePositionInches) {
    homePosition = homePositionInches;
  }

  public boolean getAtPosition(double positionInches, double tolerance) {
    var currentPosition = inputs.leftPositionInches + homePosition;
    return (positionInches - tolerance < currentPosition) && (positionInches + tolerance > currentPosition);
  }
}
