/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.OdysseusManager;
import com.team1165.robot.OdysseusState;
import com.team1165.robot.commands.drivetrain.DriveToPose;
import com.team1165.robot.globalconstants.FieldConstants.Reef;
import com.team1165.robot.globalconstants.FieldConstants.Reef.Level;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.Supplier;

public class RobotCommands {

  // Score command tunables
  private static final LoggedTunableNumber scoreEndCurrent =
      new LoggedTunableNumber("Commands/Score/EndingCurrent", 10.0);

  // Auto score command tunables
  private static final LoggedTunableNumber autoScoreFirstPoseOffset =
      new LoggedTunableNumber("Commands/AutoScore/FirstPoseOffset", -0.3);
  private static final LoggedTunableNumber autoScoreElevatorRaiseDistance =
      new LoggedTunableNumber("Commands/AutoScore/ElevatorRaiseDistance", 1.75);
  private static final LoggedTunableNumber autoScoreElevatorToleranceBeforeMoving =
      new LoggedTunableNumber("Commands/AutoScore/ElevatorToleranceBeforeMoving", 1.0);
  private static final LoggedTunableNumber autoScoreDistanceToleranceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/DistanceToleranceBeforeScore", 0.05);
  private static final LoggedTunableNumber autoScoreDistanceDebounceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/DistanceDebounceBeforeScore", 0.1);
  private static final LoggedTunableNumber autoScoreElevatorToleranceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/ElevatorToleranceBeforeScore", 0.04);

  // Zeroing command tunables
  private static final LoggedTunableNumber zeroingCurrent =
      new LoggedTunableNumber("Commands/Zeroing/Current", 20.0);

  // region Score Automation
  public static Command score(OdysseusManager robot, boolean fastScore, double timeout) {
    return new ConditionalCommand(
        robot
            .stateSupplierCommand(
                () ->
                    switch (robot.getCurrentState()) {
                      case L1 -> fastScore ? OdysseusState.FAST_SCORE_L1 : OdysseusState.SCORE_L1;
                      case L2 -> fastScore ? OdysseusState.FAST_SCORE_L2 : OdysseusState.SCORE_L2;
                      case L3 -> fastScore ? OdysseusState.FAST_SCORE_L3 : OdysseusState.SCORE_L3;
                      case L4 -> fastScore ? OdysseusState.FAST_SCORE_L4 : OdysseusState.SCORE_L4;
                      default -> OdysseusState.IDLE;
                    })
            .alongWith(
                new WaitUntilCommand(
                    () ->
                        robot.getFlywheelCurrent() > 0
                            && robot.getFlywheelCurrent() < scoreEndCurrent.get()))
            .andThen(
                robot.stateSupplierCommand(
                    () ->
                        switch (robot.getCurrentState()) {
                          case SCORE_L1, FAST_SCORE_L1 -> OdysseusState.L1;
                          case SCORE_L2, FAST_SCORE_L2 -> OdysseusState.L2;
                          case SCORE_L3, FAST_SCORE_L3 -> OdysseusState.L3;
                          case SCORE_L4, FAST_SCORE_L4 -> OdysseusState.L4;
                          default -> OdysseusState.IDLE;
                        }))
            .withTimeout(timeout),
        Commands.none(),
        () ->
            robot.getCurrentState() == OdysseusState.L1
                || robot.getCurrentState() == OdysseusState.L2
                || robot.getCurrentState() == OdysseusState.L3
                || robot.getCurrentState() == OdysseusState.L4);
  }

  public static Command score(OdysseusManager robot) {
    return score(robot, false, 0.5);
  }

  public static Command autoScore(
      OdysseusManager robot,
      Drive drive,
      Supplier<Reef.Location> face,
      Supplier<Reef.Level> level) {
    var driveCloseToFaceStart =
        new DriveToPose(
            drive,
            () ->
                face.get()
                    .getPose()
                    .transformBy(
                        new Transform2d(autoScoreFirstPoseOffset.get(), 0.0, Rotation2d.kZero)));
    var switchToHeight = setLevelState(robot, level);
    var driveToFace = new DriveToPose(drive, () -> face.get().getPose());
    var driveCloseToFaceEnd =
        new DriveToPose(
            drive,
            () ->
                face.get()
                    .getPose()
                    .transformBy(
                        new Transform2d(autoScoreFirstPoseOffset.get(), 0.0, Rotation2d.kZero)));
    var debouncer = new Debouncer(autoScoreDistanceDebounceBeforeScore.get(), DebounceType.kRising);

    return Commands.runOnce(
            () -> {
              debouncer.setDebounceTime(autoScoreDistanceDebounceBeforeScore.get());
              debouncer.calculate(false);
            })
        .alongWith(robot.stateCommand(OdysseusState.IDLE))
        .andThen(
            driveCloseToFaceStart
                .alongWith(
                    new WaitUntilCommand(
                            () ->
                                drive
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(face.get().getPose().getTranslation())
                                    < autoScoreElevatorRaiseDistance.get())
                        .andThen(switchToHeight))
                .until(() -> robot.getElevatorAtGoal(autoScoreElevatorToleranceBeforeMoving.get())))
        .andThen(driveToFace)
        .raceWith(
            new WaitUntilCommand(
                () ->
                    drive
                                .getPose()
                                .getTranslation()
                                .getDistance(face.get().getPose().getTranslation())
                            < autoScoreDistanceToleranceBeforeScore.get()
                        && robot.getElevatorAtGoal(autoScoreElevatorToleranceBeforeScore.get())))
        .andThen(score(robot))
        .andThen(
            driveCloseToFaceEnd.alongWith(robot.stateCommand(OdysseusState.IDLE)).withTimeout(0.2));
  }

  // endregion
  public static Command moveUpLevel(OdysseusManager robot) {
    return robot.stateSupplierCommand(
        () ->
            switch (robot.getCurrentState()) {
              case L1 -> OdysseusState.L2;
              case L2 -> OdysseusState.L3;
              case L3, L4 -> OdysseusState.L4;
              default -> OdysseusState.L1;
            });
  }

  public static Command moveDownLevel(OdysseusManager robot) {
    return robot.stateSupplierCommand(
        () ->
            switch (robot.getCurrentState()) {
              case L4 -> OdysseusState.L3;
              case L3 -> OdysseusState.L2;
              case L2 -> OdysseusState.L1;
              default -> OdysseusState.IDLE;
            });
  }

  public static Command setLevelState(OdysseusManager robot, Supplier<Level> level) {
    return robot.stateSupplierCommand(
        () ->
            switch (level.get()) {
              case L1 -> OdysseusState.L1;
              case L2 -> OdysseusState.L2;
              case L3 -> OdysseusState.L3;
              case L4 -> OdysseusState.L4;
            });
  }

  public static Command setScoreLevelState(OdysseusManager robot, Supplier<Level> level) {
    return robot.stateSupplierCommand(
        () ->
            switch (level.get()) {
              case L1 -> OdysseusState.SCORE_L1;
              case L2 -> OdysseusState.SCORE_L2;
              case L3 -> OdysseusState.SCORE_L3;
              case L4 -> OdysseusState.SCORE_L4;
            });
  }

  public static Command zeroElevator(OdysseusManager robot, Elevator elevator) {
    return robot
        .stateCommand(OdysseusState.ZERO_ELEVATOR)
        .alongWith(new WaitUntilCommand(() -> elevator.getCurrent() > zeroingCurrent.get()))
        .andThen(Commands.runOnce(elevator::setPositionToZero))
        .andThen(robot.stateCommand(OdysseusState.IDLE));
  }
}
