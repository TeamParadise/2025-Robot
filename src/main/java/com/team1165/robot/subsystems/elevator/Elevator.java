/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator;

import com.team1165.robot.subsystems.elevator.io.ElevatorIO;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

@Logged
public class Elevator extends SubsystemBase {

  //  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP",
  // ELEVATOR_CONFIG.Slot0.kP);
  //  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI",
  // ELEVATOR_CONFIG.Slot0.kI);
  //  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD",
  // ELEVATOR_CONFIG.Slot0.kD);
  //  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS",
  // ELEVATOR_CONFIG.Slot0.kS);
  //  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV",
  // ELEVATOR_CONFIG.Slot0.kV);
  //  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA",
  // ELEVATOR_CONFIG.Slot0.kA);
  // All can be used either for log or for simple motor feed.

  public ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  //  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;
  //  private BooleanSupplier preparerisingSupplier;

  //  public void setPreparerisingSupplier(BooleanSupplier preparerisingSupplier) {
  //    this.preparerisingSupplier = preparerisingSupplier;
  //  }

  // Disconnected alerts
  private final Alert leftDisconnected =
      new Alert("Left ElevatorMotor disconnected!", AlertType.kWarning);
  private final Alert rightDisconnected =
      new Alert("Right ElevatorMotor disconnected!", AlertType.kWarning);

  //  private boolean isDrawingHighCurrent() { Don't know what happened here
  //    return Math.abs(inputs.leftSupplyCurrentAmps) > 50.0
  //        || Math.abs(inputs.rightSupplyCurrentAmps) > 50.0;
  //  }

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
