/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction.wpilog;

import com.team1165.robot.atk.junction.LogDataReceiver;
import com.team1165.robot.atk.junction.LogReplaySource;
import com.team1165.robot.atk.junction.LogTable;
import com.team1165.robot.atk.junction.LogTable.LogValue;
import com.team1165.robot.atk.junction.LogTable.LoggableType;
import edu.wpi.first.util.datalog.DataLogIterator;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/** Replays log values from a WPILOG file. */
public class WPILOGReader implements LogReplaySource {
  private final String filename;
  private boolean isValid;

  private DataLogReader reader;
  private DataLogIterator iterator;

  private Long timestamp;
  private Map<Integer, String> entryIDs;
  private Map<Integer, LoggableType> entryTypes;
  private Map<Integer, String> entryCustomTypes;

  public WPILOGReader(String filename) {
    this.filename = filename;
  }

  public void start() {
    // Open log file
    try {
      reader = new DataLogReader(filename);
    } catch (IOException e) {
      DriverStation.reportError("[AdvantageKit] Failed to open replay log file.", true);
    }

    // Check validity
    if (!reader.isValid()) {
      DriverStation.reportError("[AdvantageKit] The replay log is not a valid WPILOG file.", false);
      isValid = false;
    } else if (!reader.getExtraHeader().equals(WPILOGConstants.extraHeader)) {
      DriverStation.reportError(
          "[AdvantageKit] The replay log was not produced by AdvantageKit.", true);
      isValid = false;
    } else {
      isValid = true;
    }

    // Create iterator and reset
    iterator = reader.iterator();
    timestamp = null;
    entryIDs = new HashMap<>();
    entryTypes = new HashMap<>();
    entryCustomTypes = new HashMap<>();
  }

  public boolean updateTable(LogTable table) {
    if (!isValid) {
      return false;
    }

    // Update timestamp (except the first cycle b/c it isn't known yet)
    if (timestamp != null) {
      table.setTimestamp(timestamp);
    }

    // Iterate over log
    boolean readError = false;
    while (iterator.hasNext()) {
      DataLogRecord record;
      try {
        record = iterator.next();
      } catch (Exception e) {
        readError = true;
        break;
      }

      if (record.isControl()) {
        if (record.isStart()) { // Ignore other control records
          if (record.getStartData().metadata.equals(WPILOGConstants.entryMetadata)) {
            entryIDs.put(record.getStartData().entry, record.getStartData().name);
            String typeStr = record.getStartData().type;
            entryTypes.put(record.getStartData().entry, LoggableType.fromWPILOGType(typeStr));
            if (typeStr.startsWith("proto:")
                || typeStr.startsWith("struct:")
                || typeStr.equals("structschema")) {
              entryCustomTypes.put(record.getStartData().entry, typeStr);
            }
          }
        }

      } else {
        String entry = entryIDs.get(record.getEntry());
        if (entry != null) {
          if (entry.equals(LogDataReceiver.timestampKey)) {
            boolean firstTimestamp = timestamp == null;
            timestamp = record.getInteger();
            if (firstTimestamp) {
              table.setTimestamp(timestamp);
            } else {
              break; // End of cycle
            }

          } else if (timestamp != null && record.getTimestamp() == timestamp) {
            entry = entry.substring(1); // Remove leading slash
            if (entry.startsWith("ReplayOutputs")) {
              // Don't retrieve old replay outputs
              continue;
            }
            String customType = entryCustomTypes.get(record.getEntry());
            switch (entryTypes.get(record.getEntry())) {
              case Raw:
                table.put(entry, new LogValue(record.getRaw(), customType));
                break;
              case Boolean:
                table.put(entry, new LogValue(record.getBoolean(), customType));
                break;
              case Integer:
                table.put(entry, new LogValue(record.getInteger(), customType));
                break;
              case Float:
                table.put(entry, new LogValue(record.getFloat(), customType));
                break;
              case Double:
                table.put(entry, new LogValue(record.getDouble(), customType));
                break;
              case String:
                table.put(entry, new LogValue(record.getString(), customType));
                break;
              case BooleanArray:
                table.put(entry, new LogValue(record.getBooleanArray(), customType));
                break;
              case IntegerArray:
                table.put(entry, new LogValue(record.getIntegerArray(), customType));
                break;
              case FloatArray:
                table.put(entry, new LogValue(record.getFloatArray(), customType));
                break;
              case DoubleArray:
                table.put(entry, new LogValue(record.getDoubleArray(), customType));
                break;
              case StringArray:
                table.put(entry, new LogValue(record.getStringArray(), customType));
                break;
            }
          }
        }
      }
    }

    // Continue if there is more data
    return iterator.hasNext() && !readError;
  }
}
