/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems;

import static com.team1165.robot.subsystems.ElevatorConstants.ELEVATOR_CONFIG;

import com.team1165.robot.Alert;
import com.team1165.robot.Constants;
import com.team1165.robot.Util.LinearProfile;
import com.team1165.robot.Util.LoggedTunableNumber;
import com.team1165.robot.atk.junction.Logger;
import com.team1165.robot.atk.junction.inputs.LoggableInputs;
import com.team1165.robot.subsystems.ElevatorIO.ElevatorIOInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

@Logged
public class Elevator extends SubsystemBase {

//  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ELEVATOR_CONFIG.Slot0.kP);
//  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ELEVATOR_CONFIG.Slot0.kI);
//  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ELEVATOR_CONFIG.Slot0.kD);
//  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", ELEVATOR_CONFIG.Slot0.kS);
//  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", ELEVATOR_CONFIG.Slot0.kV);
//  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", ELEVATOR_CONFIG.Slot0.kA);
//All can be used either for log or for simple motor feed.

  public ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  public LinearProfile leftProfile;
  public LinearProfile rightProfile;
//  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;
  //  private BooleanSupplier preparerisingSupplier;

  //  public void setPreparerisingSupplier(BooleanSupplier preparerisingSupplier) {
  //    this.preparerisingSupplier = preparerisingSupplier;
  //  }

  // Disconnected alerts
  private final Alert leftDisconnected =
      new Alert("Left ElevatorMotor disconnected!", Alert.AlertType.WARNING);
  private final Alert rightDisconnected =
      new Alert("Right ElevatorMotor disconnected!", Alert.AlertType.WARNING);



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
    Logger.processInputs("Elevator", (LoggableInputs) inputs);

    // Set alerts
    leftDisconnected.set(!inputs.leftMotorConnected);
    rightDisconnected.set(!inputs.rightMotorConnected);

    //  this i seperate but we can keep it just in case, pretty cool flywheel code
    // Check controllers
    //    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP,
    // kI, kD);
    //    LoggedTunableNumber.ifChanged(
    //        hashCode(), kSVA -> ff = new SimpleMotorFeedforward(kSVA[0], kSVA[1], kSVA[2]), kS,
    // kV, kA);
    //    LoggedTunableNumber.ifChanged(
    //        hashCode(),
    //        () -> {
    //          leftProfile.setMaxAcceleration(maxAcceleration.get());
    //          rightProfile.setMaxAcceleration(maxAcceleration.get());
    //        },
    //        maxAcceleration);

    // Stop when disabled





    //    SmartDashboard.putNumber("Elevator/Left/CLO",
    // leftMotorFollower.getClosedLoopOutput().getValueAsDouble());
    //    SmartDashboard.putNumber("Elevator/Left/Output", leftMotorFollower.get());
    //    SmartDashboard.putNumber("Elevator/Left/Inverted",
    // leftMotorFollower.getAppliedRotorPolarity().getValueAsDouble());
    //    SmartDashboard.putNumber("Elevator/Left/Current",
    // leftMotorFollower.getSupplyCurrent().getValueAsDouble());
    //
    //    SmartDashboard.putNumber("Elevator/Right/CLO",
    // rightMotorLeader.getClosedLoopOutput().getValueAsDouble());
    //    SmartDashboard.putNumber("Elevator/Right/Output", rightMotorLeader.get());
    //    SmartDashboard.putNumber("Elevator/Right/Inverted",
    // rightMotorLeader.getAppliedRotorPolarity().getValueAsDouble());
    //    SmartDashboard.putNumber("Elevator/Right/Current",
    // rightMotorLeader.getSupplyCurrent().getValueAsDouble());
  }



  /** Runs Elevator at the commanded voltage or amps. */
  //  public void runCharacterization(double input) {
  //    setGoal(Goal.CHARACTERIZING);
  //    io.runCharacterizationLeft(input);
  //    io.runCharacterizationRight(input);
  //  }
  //
  //  /** Get characterization velocity */
  //  public double getCharacterizationVelocity() {
  //    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  //  }
  //
  //  /** Get if velocity profile has ended */
  //  public void logElevatorAtGoal() {
  //    Logger.recordOutput("Elevators/AtGoal", goal); // Where 'atGoal' is a boolean variable
  //  }
  //
  //  public boolean atGoal() {
  //    return goal == Goal.IDLE
  //        || (leftProfile.getCurrentSetpoint() == goal.getLeftGoal()
  //            && rightProfile.getCurrentSetpoint() == goal.getRightGoal());
  //  }
  //
  //  public Command risingCommand() {
  //    return startEnd(() -> setGoal(Goal.rising), () -> setGoal(Goal.IDLE))
  //        .withName("Elevator rising");
  //  }
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

  public void setPosition(Double height, double velocity) {
    io.setPosition(height, velocity);
  }
}
