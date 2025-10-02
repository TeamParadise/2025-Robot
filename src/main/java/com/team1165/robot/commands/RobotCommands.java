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
import com.team1165.robot.commands.drivetrain.DriveToPoseProfiled;
import com.team1165.robot.globalconstants.FieldConstants;
import com.team1165.robot.globalconstants.FieldConstants.Reef;
import com.team1165.robot.globalconstants.FieldConstants.Reef.Level;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.util.commands.ChezySequenceCommandGroup;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotCommands {

  // Score command tunables
  private static final LoggedTunableNumber scoreEndCurrent =
      new LoggedTunableNumber("Commands/Score/EndingCurrent", 10.0);

  // Auto score command tunables
  private static final LoggedTunableNumber autoScoreFirstPoseOffset =
      new LoggedTunableNumber("Commands/AutoScore/FirstPoseOffset", -0.3);
  private static final LoggedTunableNumber autoScoreElevatorRaiseDistance =
      new LoggedTunableNumber("Commands/AutoScore/ElevatorRaiseDistance", 2.0);
  private static final LoggedTunableNumber autoScoreElevatorToleranceBeforeMoving =
      new LoggedTunableNumber("Commands/AutoScore/ElevatorToleranceBeforeMoving", 0.75);
  private static final LoggedTunableNumber autoScorePrimaryDistanceToleranceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/PrimaryDistanceTolerance", 0.04);
  private static final LoggedTunableNumber autoScorePrimaryDistanceDebounceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/PrimaryDistanceDebounceBeforeScore", 0.06);
  private static final LoggedTunableNumber autoScoreSecondaryDistanceToleranceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/SecondaryDistanceToleranceBeforeScore", 0.09);
  private static final LoggedTunableNumber autoScoreSecondaryDistanceDebounceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/SecondaryDistanceDebounceBeforeScore", 0.25);
  private static final LoggedTunableNumber autoScoreElevatorToleranceBeforeScore =
      new LoggedTunableNumber("Commands/AutoScore/ElevatorToleranceBeforeScore", 0.15);
  private static final LoggedTunableNumber autoScoreClosePoseOffset =
      new LoggedTunableNumber("Commands/AutoScore/ClosePoseOffset", 0.06);

  // Zeroing command tunables
  private static final LoggedTunableNumber zeroingCurrent =
      new LoggedTunableNumber("Commands/Zeroing/Current", 20.0);

  // region Score Automation
  public static Command score(OdysseusManager robot, boolean fastScore, double timeout) {
    return new ConditionalCommand(
        new ChezySequenceCommandGroup(
                robot
                    .stateSupplierCommand(
                        () ->
                            switch (robot.getCurrentState()) {
                              case L1, SCORE_L1, FAST_SCORE_L1 ->
                                  fastScore ? OdysseusState.FAST_SCORE_L1 : OdysseusState.SCORE_L1;
                              case L2, SCORE_L2, FAST_SCORE_L2 ->
                                  fastScore ? OdysseusState.FAST_SCORE_L2 : OdysseusState.SCORE_L2;
                              case L3, SCORE_L3, FAST_SCORE_L3 ->
                                  fastScore ? OdysseusState.FAST_SCORE_L3 : OdysseusState.SCORE_L3;
                              case L4, SCORE_L4, FAST_SCORE_L4 ->
                                  fastScore ? OdysseusState.FAST_SCORE_L4 : OdysseusState.SCORE_L4;
                              default -> OdysseusState.IDLE;
                            })
                    .alongWith(
                        new WaitUntilCommand(
                            () ->
                                robot.getFlywheelCurrent() > 0
                                    && robot.getFlywheelCurrent() < scoreEndCurrent.get())),
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
        () -> {
          var state = robot.getCurrentState();
          return state == OdysseusState.L1
              || state == OdysseusState.SCORE_L1
              || state == OdysseusState.FAST_SCORE_L1
              || state == OdysseusState.L2
              || state == OdysseusState.SCORE_L2
              || state == OdysseusState.FAST_SCORE_L2
              || state == OdysseusState.L3
              || state == OdysseusState.SCORE_L3
              || state == OdysseusState.FAST_SCORE_L3
              || state == OdysseusState.L4
              || state == OdysseusState.SCORE_L4
              || state == OdysseusState.FAST_SCORE_L4;
        });
  }

  public static Command score(OdysseusManager robot) {
    return score(robot, false, 0.5);
  }

  public static Command autoScore(
      OdysseusManager robot,
      Drive drive,
      Supplier<Reef.Location> face,
      Supplier<Reef.Level> level) {
    // Create debouncer and boolean supplier used in the commands
    var primaryDebouncer =
        new Debouncer(autoScorePrimaryDistanceDebounceBeforeScore.get(), DebounceType.kRising);
    var secondaryDebouncer =
        new Debouncer(autoScoreSecondaryDistanceDebounceBeforeScore.get(), DebounceType.kRising);
    BooleanSupplier readyToRaiseElevator =
        () ->
            drive.getPose().getTranslation().getDistance(face.get().getPose().getTranslation())
                < autoScoreElevatorRaiseDistance.get();
    BooleanSupplier readyToMoveCloseToReef =
        () -> robot.getElevatorAtGoal(autoScoreElevatorToleranceBeforeMoving.get());
    BooleanSupplier readyToScore =
        () -> {
          double distance =
              drive.getPose().getTranslation().getDistance(face.get().getPose().getTranslation());
          return (primaryDebouncer.calculate(
                      distance < autoScorePrimaryDistanceToleranceBeforeScore.get())
                  || secondaryDebouncer.calculate(
                      distance < autoScoreSecondaryDistanceToleranceBeforeScore.get()))
              && robot.getElevatorAtGoal(autoScoreElevatorToleranceBeforeScore.get());
        };

    // Commands
    var initializationCommand =
        // Set robot state to idle if not ready to raise the elevator already
        new ConditionalCommand(
            robot.stateCommand(OdysseusState.IDLE), Commands.none(), readyToRaiseElevator) {
          // Reset debouncer when the auto score command starts
          @Override
          public void initialize() {
            super.initialize();
            primaryDebouncer.setDebounceTime(autoScorePrimaryDistanceDebounceBeforeScore.get());
            primaryDebouncer.calculate(false);
            secondaryDebouncer.setDebounceTime(autoScoreSecondaryDistanceDebounceBeforeScore.get());
            secondaryDebouncer.calculate(false);
          }
        };
    var driveToInitialReefPosition =
        // Drive close to the reef until elevator is safe to move all the way to the reef
        new DriveToPoseProfiled(
                drive,
                () ->
                    face.get()
                        .getPose()
                        .transformBy(
                            new Transform2d(autoScoreFirstPoseOffset.get(), 0.0, Rotation2d.kZero)))
            .withDeadline(
                // Wait until ready to raise the elevator, raise elevator, wait until safe to move
                new ChezySequenceCommandGroup(
                    new WaitUntilCommand(readyToRaiseElevator),
                    setLevelState(robot, level),
                    new WaitUntilCommand(readyToMoveCloseToReef)));
    var driveAndScore =
        // Drive to final scoring position and score once ready
        new DriveToPoseProfiled(drive, () -> face.get().getPose())
            .withDeadline(
                new ChezySequenceCommandGroup(
                    new WaitUntilCommand(readyToScore), score(robot, false, 0.35)));
    var scoreFallback =
        // If the coral was not fully scored, score at a faster speed and move forward slightly
        new ConditionalCommand(
            score(robot, true, 0.75)
                .deadlineFor(
                    new DriveToPose(
                        drive,
                        () ->
                            face.get()
                                .getPose()
                                .transformBy(
                                    new Transform2d(
                                        autoScoreClosePoseOffset.get(), 0.0, Rotation2d.kZero)))),
            Commands.none(),
            () -> {
              var state = robot.getCurrentState();
              return state != OdysseusState.L1
                  && state != OdysseusState.L2
                  && state != OdysseusState.L3
                  && state != OdysseusState.L4;
            });

    // Assemble commands into group
    return new ChezySequenceCommandGroup(
        initializationCommand, driveToInitialReefPosition, driveAndScore, scoreFallback);
  }

  public static Command oldAutoScore(
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
    var driveCloseToFaceStart2 =
        new DriveToPose(
            drive,
            () ->
                face.get()
                    .getPose()
                    .transformBy(
                        new Transform2d(autoScoreFirstPoseOffset.get(), 0.0, Rotation2d.kZero)));
    var switchToHeight = setLevelState(robot, level);
    var driveToFace = new DriveToPose(drive, () -> face.get().getPose());
    var driveCloserToFace =
        new DriveToPose(
            drive,
            () ->
                face.get()
                    .getPose()
                    .transformBy(
                        new Transform2d(autoScoreClosePoseOffset.get(), 0.0, Rotation2d.kZero)));
    var debouncer =
        new Debouncer(autoScoreSecondaryDistanceDebounceBeforeScore.get(), DebounceType.kRising);

    return new ChezySequenceCommandGroup(
        Commands.runOnce(
                () -> {
                  debouncer.setDebounceTime(autoScoreSecondaryDistanceDebounceBeforeScore.get());
                  debouncer.calculate(false);
                })
            .alongWith(robot.stateCommand(OdysseusState.IDLE)),
        driveCloseToFaceStart.raceWith(
            new ChezySequenceCommandGroup(
                new WaitUntilCommand(
                    () ->
                        drive
                                .getPose()
                                .getTranslation()
                                .getDistance(face.get().getPose().getTranslation())
                            < autoScoreElevatorRaiseDistance.get()),
                switchToHeight)),
        driveCloseToFaceStart2.until(
            () ->
                switchToHeight.isFinished()
                    && robot.getElevatorAtGoal(autoScoreElevatorToleranceBeforeMoving.get())),
        driveToFace.withDeadline(
            new ChezySequenceCommandGroup(
                new WaitUntilCommand(
                    () ->
                        debouncer.calculate(
                                drive
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(face.get().getPose().getTranslation())
                                    < autoScoreSecondaryDistanceToleranceBeforeScore.get())
                            && robot.getElevatorAtGoal(
                                autoScoreElevatorToleranceBeforeScore.get())),
                score(robot, false, 0.35))),
        new ConditionalCommand(
            score(robot, true, 0.75).deadlineFor(driveCloserToFace),
            Commands.none(),
            () -> {
              var state = robot.getCurrentState();
              return state != OdysseusState.L1
                  && state != OdysseusState.L2
                  && state != OdysseusState.L3
                  && state != OdysseusState.L4;
            }));
  }

  // region Auto Align
  public static Command autoAlignToNearest(Drive drive, boolean left) {
    return Commands.defer(
        () -> {
          Pose2d closestAprilTag = drive.getPose().nearest(FieldConstants.reefAprilTagPoses);
          int closestID =
              FieldConstants.reefAprilTagIDs.get(
                  FieldConstants.reefAprilTagPoses.indexOf(closestAprilTag));

          return new DriveToPose(
              drive,
              () ->
                  switch (closestID) {
                    case 7, 18 -> left ? Reef.Location.A.getPose() : Reef.Location.B.getPose();
                    case 8, 17 -> left ? Reef.Location.C.getPose() : Reef.Location.D.getPose();
                    case 9, 22 -> left ? Reef.Location.E.getPose() : Reef.Location.F.getPose();
                    case 10, 21 -> left ? Reef.Location.G.getPose() : Reef.Location.H.getPose();
                    case 11, 20 -> left ? Reef.Location.I.getPose() : Reef.Location.J.getPose();
                    default -> left ? Reef.Location.K.getPose() : Reef.Location.L.getPose();
                  });
        },
        Set.of(drive));
  }

  // endregion

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
        .alongWith(
            new WaitUntilCommand(() -> elevator.getCurrent() > zeroingCurrent.get())
                .andThen(Commands.runOnce(elevator::setPositionToZero)))
        .andThen(robot.stateCommand(OdysseusState.IDLE));
  }
}
