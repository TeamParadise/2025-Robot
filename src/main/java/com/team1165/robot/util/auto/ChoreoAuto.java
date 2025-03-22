/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team1165.robot.FieldConstants.*;
import com.team1165.robot.FieldConstants.Reef.Location;
import com.team1165.robot.commands.AutoScore;
import com.team1165.robot.commands.Intake;
import com.team1165.robot.commands.drivetrain.DriveToPose;
import com.team1165.robot.commands.elevator.ElevatorPosition;
import com.team1165.robot.commands.flywheels.FlywheelsPercenmt;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.flywheels.Flywheels;
import com.team1165.robot.subsystems.funnel.Funnel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Class that represents a full Choreo auto. */
public class ChoreoAuto {
  private final Time delayBeforeStart;
  private final boolean pushPartner;
  private final AutoSegmentConfig[] segments;
  private static final double translationToleranceBeforeScoring = 0.4;
  private static final double rotationToleranceBeforeScoring = 0.78539816339;

  public ChoreoAuto(Time delayBeforeStart, boolean pushPartner, AutoSegmentConfig... segments) {
    this.delayBeforeStart = delayBeforeStart;
    this.pushPartner = pushPartner;
    this.segments = segments;
  }

  public ChoreoAuto(Time delayBeforeStart, AutoSegmentConfig... segments) {
    this(delayBeforeStart, false, segments);
  }

  public ChoreoAuto(boolean pushPartner, AutoSegmentConfig... segments) {
    this(Seconds.zero(), pushPartner, segments);
  }

  public ChoreoAuto(AutoSegmentConfig... segments) {
    this(Seconds.zero(), false, segments);
  }

  public Command getAutoCommand(
      Drive drive, Elevator elevator, Flywheels flywheels, Funnel funnel) {
    // Check to make sure there are actually segments in this auto
    if (segments.length != 0) {
      // Create the routine to be used with this auto
      AutoRoutine routine =
          drive.getAutoFactory().newRoutine("Starting at " + segments[0].reefLocation());

      // Get the starting, flipped reef location, if we need to flip (for A, H, I, J, K, L)
      Location flippedOverX = segments[0].reefLocation().getFlippedReef();
      boolean flipped = !(flippedOverX == segments[0].reefLocation());

      // Create the initial scoring trajectory (from the starting line), with flip and push partner
      AutoTrajectory scoringTrajectory =
          flipped
              ? routine.trajectory(
                  ChoreoUtils.flipOverX(
                      routine
                          .trajectory("SL to " + flippedOverX.name() + (pushPartner ? " Push" : ""))
                          .getRawTrajectory(),
                      "SL to " + segments[0].reefLocation().name() + (pushPartner ? " Push" : "")))
              : routine.trajectory("SL to " + flippedOverX.name() + (pushPartner ? " Push" : ""));

      // Create the coral station trajectory
      AutoTrajectory coralStationTrajectory = buildCoralStation(routine, segments[0]);

      // Create the full trajectory, will be reassigned later to add more commands onto the schedule
      Command lastFullTrajectory =
          buildFullTrajectory(
              drive,
              elevator,
              flywheels,
              funnel,
              scoringTrajectory,
              coralStationTrajectory,
              segments[0]);

      // Loop through the rest of the segments
      for (int i = 1; i < segments.length; i++) {
        // Create the new scoring trajectory, with flipping as necessary
        var newTraj =
            buildBaseScoringTrajectory(routine, segments[i - 1].coralStation(), segments[i]);
        // Create the new coral station trajectory, with flipping as necessary
        var newCsTraj = buildCoralStation(routine, segments[i]);
        // Add the new trajectory onto the last trajectory that ran
        lastFullTrajectory =
            lastFullTrajectory.andThen(
                buildFullTrajectory(
                    drive, elevator, flywheels, funnel, newTraj, newCsTraj, segments[i]));
      }

      // Start the routine and trajectory
      routine
          .active()
          .onTrue(
              Commands.sequence(
                  scoringTrajectory.resetOdometry(),
                  new WaitCommand(delayBeforeStart),
                  lastFullTrajectory));

      return routine.cmd();
    } else {
      return Commands.none(); // If no segments, just don't do anything.
    }
  }

  public Command getSimpleAutoCommand(
      Drive drive, Elevator elevator, Flywheels flywheels, Funnel funnel) {
    var initialDriveCommand =
        new DriveToPose(
            drive,
            () ->
                segments[0]
                    .reefLocation()
                    .getPose()
                    .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)));
    var nextDriveCommand = new DriveToPose(drive, () -> segments[0].reefLocation().getPose());

    return new ElevatorPosition(elevator, () -> segments[0].reefLevel().getElevatorHeight())
        .alongWith(
            initialDriveCommand
                .until(() -> elevator.getAtPosition(12.5, 1))
                .andThen(
                    nextDriveCommand
                        .alongWith(
                            new WaitCommand(2).andThen(new FlywheelsPercenmt(flywheels, () -> 0.3)))
                        .withTimeout(3)))
        .withTimeout(10)
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        segments[0]
                            .reefLocation()
                            .getPose()
                            .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)))
                .alongWith(new ElevatorPosition(elevator, () -> 0)));
  }

  public static AutoTrajectory buildBaseScoringTrajectory(
      AutoRoutine routine,
      CoralStationLocation startingCoralStation,
      AutoSegmentConfig mainSequence) {
    // Get the flipped reef location, if we need to flip (for A, H, I, J, K, L)
    Location flippedOverX = mainSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == mainSequence.reefLocation());

    // Get the starting coral station, and flip if needed
    CoralStationLocation flippedStartingCs =
        flipped
            ? (startingCoralStation == CoralStationLocation.RCS
                ? CoralStationLocation.LCS
                : CoralStationLocation.RCS)
            : startingCoralStation;

    // Create and return trajectory, with flipping as needed
    return flipped
        ? routine.trajectory(
            ChoreoUtils.flipOverX(
                routine
                    .trajectory(flippedStartingCs.name() + " to " + flippedOverX.name())
                    .getRawTrajectory(),
                startingCoralStation.name() + " to " + mainSequence.reefLocation().name()))
        : routine.trajectory(flippedStartingCs.name() + " to " + flippedOverX.name());
  }

  public static Command buildFullTrajectory(
      Drive drive,
      Elevator elevator,
      Flywheels flywheels,
      Funnel funnel,
      AutoTrajectory scoringTrajectory,
      AutoTrajectory coralStationTrajectory,
      AutoSegmentConfig mainSequence) {
    // Get final poses, auto score command, and intake command
    var scoringFinalPose = scoringTrajectory.getRawTrajectory().getFinalPose(false);
    var coralFinalPose = coralStationTrajectory.getRawTrajectory().getFinalPose(false);
    var autoScore =
        new AutoScore(
            mainSequence.reefLocation(), mainSequence.reefLevel(), drive, elevator, flywheels);
    var intake =
        new Intake(elevator, flywheels, funnel)
            .andThen(new WaitCommand(mainSequence.delayAfterIntake()));
    var moveElevatorUp =
        new ElevatorPosition(elevator, () -> mainSequence.reefLevel().getElevatorHeight());

    // Run the scoring command when close to final location, and run basic PID to pose after the
    // trajectory ends to ensure alignment. Also run coral station trajectory once done.
    scoringFinalPose.ifPresent(pose2d -> {});

    coralFinalPose.ifPresent(
        pose2d -> {
          coralStationTrajectory.atPose(pose2d, 0.7, 3).onTrue(intake);
          coralStationTrajectory
              .done()
              .onTrue(
                  new DriveToPose(drive, () -> mainSequence.coralStation().getPose())
                      .until(intake::isFinished));
        });

    return scoringTrajectory
        .cmd()
        .andThen(
            moveElevatorUp.alongWith(
                new WaitCommand(0.7)
                    .andThen(
                        new DriveToPose(drive, () -> mainSequence.reefLocation().getPose())
                            .withTimeout(1.10)
                            .andThen(
                                new FlywheelsPercenmt(flywheels, () -> 0.3).withTimeout(0.5)))))
        .withTimeout(4)
        .andThen(
            new ElevatorPosition(elevator, () -> 0)
                .alongWith(new DriveToPose(drive, () -> scoringTrajectory.getFinalPose().get()))
                .withTimeout(0.5))
        .andThen(coralStationTrajectory.cmd())
        .until(intake::isFinished);
  }

  public static AutoTrajectory buildCoralStation(
      AutoRoutine routine, AutoSegmentConfig mainSequence) {
    // Get the flipped reef location
    Location flippedOverX = mainSequence.reefLocation().getFlippedReef();
    boolean flipped = !(flippedOverX == mainSequence.reefLocation());

    // Get the flipped position of the coral station, if needed
    CoralStationLocation flippedCs =
        flipped
            ? (mainSequence.coralStation() == CoralStationLocation.RCS
                ? CoralStationLocation.LCS
                : CoralStationLocation.RCS)
            : mainSequence.coralStation();

    // Create the trajectory, flipped if required
    return flipped
        ? routine.trajectory(
            ChoreoUtils.flipOverX(
                routine
                    .trajectory(flippedOverX.name() + " to " + flippedCs.name())
                    .getRawTrajectory(),
                mainSequence.reefLocation().name() + " to " + mainSequence.coralStation()))
        : routine.trajectory(flippedOverX.name() + " to " + flippedCs.name());
  }
}
