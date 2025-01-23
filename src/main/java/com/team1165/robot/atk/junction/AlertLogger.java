/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unchecked")
class AlertLogger {
  private static Map<String, Object> groups = null;
  private static Map<String, StringArraySubscriber> errorSubscribers = new HashMap<>();
  private static Map<String, StringArraySubscriber> warningSubscribers = new HashMap<>();
  private static Map<String, StringArraySubscriber> infoSubscribers = new HashMap<>();

  static {
    try {
      Class<?> sendableAlertsClass = Class.forName("edu.wpi.first.wpilibj.Alert$SendableAlerts");
      Field groupsField = sendableAlertsClass.getDeclaredField("groups");
      groupsField.setAccessible(true);
      groups = (Map<String, Object>) groupsField.get(null);
    } catch (ClassNotFoundException
        | IllegalArgumentException
        | IllegalAccessException
        | NoSuchFieldException
        | SecurityException e) {
      e.printStackTrace();
    }
  }

  /** Log the current state of all alerts as outputs. */
  public static void periodic() {
    if (groups == null) return;
    for (String group : groups.keySet()) {
      com.team1165.robot.atk.junction.Logger.recordOutput(group + "/.type", "Alerts");

      // Create NetworkTables subscribers
      if (!errorSubscribers.containsKey(group)) {
        errorSubscribers.put(
            group,
            NetworkTableInstance.getDefault()
                .getStringArrayTopic("/SmartDashboard/" + group + "/errors")
                .subscribe(new String[0]));
      }
      if (!warningSubscribers.containsKey(group)) {
        warningSubscribers.put(
            group,
            NetworkTableInstance.getDefault()
                .getStringArrayTopic("/SmartDashboard/" + group + "/warnings")
                .subscribe(new String[0]));
      }
      if (!infoSubscribers.containsKey(group)) {
        infoSubscribers.put(
            group,
            NetworkTableInstance.getDefault()
                .getStringArrayTopic("/SmartDashboard/" + group + "/infos")
                .subscribe(new String[0]));
      }

      // Get values
      com.team1165.robot.atk.junction.Logger.recordOutput(
          group + "/errors", errorSubscribers.get(group).get());
      com.team1165.robot.atk.junction.Logger.recordOutput(
          group + "/warnings", warningSubscribers.get(group).get());
      Logger.recordOutput(group + "/infos", infoSubscribers.get(group).get());
    }
  }
}
