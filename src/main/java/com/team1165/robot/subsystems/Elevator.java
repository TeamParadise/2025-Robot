/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems;

import static com.team1165.robot.subsystems.ElevatorKraken.gains;

import com.team1165.robot.Alert;
import com.team1165.robot.Constants;
import com.team1165.robot.subsystems.ElevatorIO.ElevatorIOInputs;
import com.team1165.robot.util.LinearProfile;
import com.team1165.robot.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", gains.kA());
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", gains.kG());

  private static final LoggedTunableNumber risingLeftRPM =
      new LoggedTunableNumber("Elevator/risingingLeftRpm", 5066.0);
  private static final LoggedTunableNumber risingRightRPM =
      new LoggedTunableNumber("Elevator/risingingRightRpm", 5066.0);
  private static final LoggedTunableNumber preparerisingMultiplier =
      new LoggedTunableNumber("Flywheels/PreparerisingMultiplier", 1.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/MaxAccelerationRpmPerSec", 9000.0);

  public ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  public LinearProfile leftProfile;
  public LinearProfile rightProfile;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
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

  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    rising(risingLeftRPM, risingRightRPM),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    Goal(DoubleSupplier leftGoal, DoubleSupplier rightGoal) {
      this.leftGoal = leftGoal;
      this.rightGoal = rightGoal;
    }

    private double getLeftGoal() {
      return leftGoal.getAsDouble();
    }

    private double getRightGoal() {
      return rightGoal.getAsDouble();
    }
  }

  public enum IdleMode {
    TELEOP,
    AUTO
  }

  private Goal goal = Goal.IDLE;

  public void logElevatorGoal() {
    Logger.recordOutput("Elevator/Goal", goal); // Replace Logger.log with your logging utility
  }

  public Goal getGoal() {
    return goal;
  }

  //  private boolean isDrawingHighCurrent() { Don't know what happened here
  //    return Math.abs(inputs.leftSupplyCurrentAmps) > 50.0
  //        || Math.abs(inputs.rightSupplyCurrentAmps) > 50.0;
  //  }

  public Elevator(ElevatorIO io) {
    this.io = io;

    leftProfile = new LinearProfile(maxAcceleration.get(), Constants.loopPeriodSecs);
    rightProfile = new LinearProfile(maxAcceleration.get(), Constants.loopPeriodSecs);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Elevator Idle"));
  }

  @Override
  public void periodic() {
    //    io.updateInputs(inputs);
    //    Logger.processInputs("Elevator", (LoggableInputs) inputs); - didn't work when build

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
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // this i seperate but we can keep it just in case, pretty cool flywheel code
    //    // Check if profile needs to be reset
    //    if (!closedLoop && wasClosedLoop) {
    //      leftProfile.reset();
    //      rightProfile.reset();
    //      wasClosedLoop = false;
    //    }
    //
    //    // Get goal
    //    double leftGoal = goal.getLeftGoal();
    //    double rightGoal = goal.getRightGoal();
    //    //    boolean idlePreparerising = goal == Goal.IDLE &&
    // preparerisingSupplier.getAsBoolean();
    //    //    if (idlePreparerising) {
    //    //      leftGoal = Goal.rising.getLeftGoal() * preparerisingMultiplier.get();
    //    //      rightGoal = Goal.rising.getRightGoal() * preparerisingMultiplier.get();
    //    //    }
    //
    //    // Run to setpoint
    //    if (closedLoop) { // supposed to put a || idlePreparerising
    //      // Update goals
    //      leftProfile.setGoal(leftGoal);
    //      rightProfile.setGoal(rightGoal);
    //      double leftSetpoint = leftProfile.calculateSetpoint();
    //      double rightSetpoint = rightProfile.calculateSetpoint();
    //      io.runVelocity(
    //          leftSetpoint, rightSetpoint, ff.calculate(leftSetpoint),
    // ff.calculate(rightSetpoint));
    //      //   RobotState.getInstance().setFlywheelAccelerating(!atGoal() ||
    // isDrawingHighCurrent()); -
    //      // dont know if we have a robot location updater.
    //    } else if (goal == Goal.IDLE) {
    //      //   RobotState.getInstance().setFlywheelAccelerating(false);
    //      io.stop();
    //    }
    //
    //    Logger.recordOutput("Elevator/SetpointLeftRpm", leftProfile.getCurrentSetpoint());
    //    Logger.recordOutput("Elevator/SetpointRightRpm", rightProfile.getCurrentSetpoint());
    //    Logger.recordOutput("Elevator/GoalLeftRpm", leftGoal);
    //    Logger.recordOutput("Elevator/GoalRightRpm", rightGoal);

    //    SmartDashboard.putNumber("Elevator/Left/CLO",
    // leftMotorFollower.getClosedLoopOutput().getValueAsDouble()); - WE NEED JUST FIGRURING OUT IF
    // IO CAN TAKE PERIODIC OR IF I NEED TO REINSTATE MOTORS
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

  /** Set the current goal of the flywheel */
  private void setGoal(Goal goal) {
    if (goal == Goal.CHARACTERIZING || goal == Goal.IDLE) {
      wasClosedLoop = closedLoop;
      closedLoop = false;
      this.goal = goal;
      return; // Don't set a goal
    }
    // If not already controlling to requested goal
    // set closed loop false
    closedLoop = this.goal == goal;
    // Enable close loop
    if (!closedLoop) {
      leftProfile.setGoal(goal.getLeftGoal(), inputs.leftVelocityRpm);
      rightProfile.setGoal(goal.getRightGoal(), inputs.rightVelocityRpm);
      closedLoop = true;
    }
    this.goal = goal;
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
    this.io.resetSensorPosition(setpoint);
  }

  /** Run both motors at voltage */
  public void setPosition(Distance height) {
    this.io.setPosition(height);
  }

  /** Stop both Elevator Motors */
  public void stop() {
    this.io.stop();
  }

  public void setPID(double kp, double ki, double kd) {
    io.setPID(kp, ki, kd);
  }

  public void setPosition(Double height, double velocity) {
    io.setPosition(height, velocity);
  }
}
