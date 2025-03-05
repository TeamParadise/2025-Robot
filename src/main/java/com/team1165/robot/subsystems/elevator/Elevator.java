/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator;

import com.team1165.robot.subsystems.elevator.constants.ElevatorConstants;
import com.team1165.robot.subsystems.elevator.io.ElevatorIO;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import com.team1165.robot.util.LoggedTunableNumber;
import edu.wpi.first.units.measure.Distance;
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

  public void resetSensorPosition(Distance setpoint) {
    io.resetSensorPosition(setpoint);
  }

  /** Run both motors at voltage */
  public void setPosition(Distance height) {
    io.setPosition(height);
  }

  /** Stop both Elevator Motors */
  public void stop() {
    io.stop();
  }

  public void setPID(double kp, double ki, double kd) {
    io.setPID(kp, ki, kd);
  }

  public void runVolts(double leftVoltage, double rightVoltage) {
    io.runVolts(leftVoltage, rightVoltage);
  }

  public void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    io.runVelocity(leftRpm, rightRpm, leftFeedforward, rightFeedforward);
  }

  public void setPosition(Double height, double velocity) {
    io.setPosition(height, velocity);
  }

  public double getLastDesiredPosition() {
    return io.getLastDesiredPosition();
  }
}
