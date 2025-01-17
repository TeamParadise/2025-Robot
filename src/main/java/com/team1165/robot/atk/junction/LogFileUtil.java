/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;

public class LogFileUtil {
  static final String environmentVariable = "AKIT_LOG_PATH";
  private static final String advantageScopeFileName = "akit-log-path.txt";

  private LogFileUtil() {}

  /**
   * Adds a suffix to the given path (e.g. "test.wpilog" -> "test_sim.wpilog").
   *
   * <p>If the input path already contains the suffix, an index will be added instead.
   */
  public static String addPathSuffix(String path, String suffix) {
    String[] tokens = path.split("\\.");
    if (tokens[0].endsWith(suffix)) {
      return tokens[0] + "_2." + tokens[1];
    } else if (tokens[0].matches(".+" + suffix + "_[0-9]+$")) {
      int splitIndex = tokens[0].lastIndexOf("_");
      int index = Integer.parseInt(tokens[0].substring(splitIndex + 1));
      return tokens[0].substring(0, splitIndex)
          + "_"
          + Integer.toString(index + 1)
          + "."
          + tokens[1];
    } else {
      return tokens[0] + suffix + "." + tokens[1];
    }
  }

  /**
   * Finds the path to a log file for replay, using the following priorities:
   *
   * <p>1. The value of the "AKIT_LOG_PATH" environment variable, if set 2. The file currently open
   * in AdvantageScope, if available 3. The result of the prompt displayed to the user
   */
  public static String findReplayLog() {
    // Read environment variables
    String envPath = findReplayLogEnvVar();
    if (envPath != null) {
      System.out.println(
          "[AdvantageKit] Replaying log from "
              + environmentVariable
              + " environment variable: \""
              + envPath
              + "\"");
      return envPath;
    }

    // Read file from AdvantageScope
    String advantageScopeLogPath = findReplayLogAdvantageScope();
    if (advantageScopeLogPath != null) {
      System.out.println(
          "[AdvantageKit] Replaying log from AdvantageScope: \"" + advantageScopeLogPath + "\"");
      return advantageScopeLogPath;
    }

    // Prompt on stdin
    System.out.print(
        "No log provided with the "
            + environmentVariable
            + " environment variable or through AdvantageScope. Enter path to file: ");
    String filename = findReplayLogUser();
    if (filename.charAt(0) == '\'' || filename.charAt(0) == '"') {
      filename = filename.substring(1, filename.length() - 1);
    }
    return filename;
  }

  /** Read the replay log from the environment variable. */
  static String findReplayLogEnvVar() {
    return System.getenv(environmentVariable);
  }

  /** Read the replay log from AdvantageScope. */
  static String findReplayLogAdvantageScope() {
    Path advantageScopeTempPath =
        Paths.get(System.getProperty("java.io.tmpdir"), advantageScopeFileName);
    String advantageScopeLogPath = null;
    try (Scanner fileScanner = new Scanner(advantageScopeTempPath)) {
      advantageScopeLogPath = fileScanner.nextLine();
    } catch (IOException e) {
    }
    return advantageScopeLogPath;
  }

  /** Read the replay log from the user. */
  static String findReplayLogUser() {
    Scanner scanner = new Scanner(System.in);
    String filename = scanner.nextLine();
    scanner.close();
    return filename;
  }
}
