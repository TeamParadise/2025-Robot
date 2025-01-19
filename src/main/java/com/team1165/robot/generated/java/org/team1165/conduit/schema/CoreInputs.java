/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

// automatically generated by the FlatBuffers compiler, do not modify

package com.team1165.robot.generated.java.org.team1165.conduit.schema;

import com.google.flatbuffers.BaseVector;
import com.google.flatbuffers.FlatBufferBuilder;
import com.google.flatbuffers.Struct;
import java.nio.ByteBuffer;

@SuppressWarnings("unused")
public final class CoreInputs extends Struct {
  public void __init(int _i, ByteBuffer _bb) {
    __reset(_i, _bb);
  }

  public CoreInputs __assign(int _i, ByteBuffer _bb) {
    __init(_i, _bb);
    return this;
  }

  public long timestamp() {
    return bb.getLong(bb_pos + 0);
  }

  public void mutateTimestamp(long timestamp) {
    bb.putLong(bb_pos + 0, timestamp);
  }

  public DSData ds() {
    return ds(new com.team1165.robot.generated.java.org.team1165.conduit.schema.DSData());
  }

  public com.team1165.robot.generated.java.org.team1165.conduit.schema.DSData ds(
      com.team1165.robot.generated.java.org.team1165.conduit.schema.DSData obj) {
    return obj.__assign(bb_pos + 8, bb);
  }

  public PDPData pdp() {
    return pdp(new com.team1165.robot.generated.java.org.team1165.conduit.schema.PDPData());
  }

  public com.team1165.robot.generated.java.org.team1165.conduit.schema.PDPData pdp(
      com.team1165.robot.generated.java.org.team1165.conduit.schema.PDPData obj) {
    return obj.__assign(bb_pos + 2304, bb);
  }

  public SystemData sys() {
    return sys(new SystemData());
  }

  public SystemData sys(SystemData obj) {
    return obj.__assign(bb_pos + 2560, bb);
  }

  public static int createCoreInputs(
      FlatBufferBuilder builder,
      long timestamp,
      int ds_allianceStation,
      int[] ds_eventName,
      int ds_gameSpecificMessageSize,
      int[] ds_gameSpecificMessage,
      int ds_matchNumber,
      int ds_replayNumber,
      int ds_matchType,
      int ds_controlWord,
      double ds_matchTime,
      int[][] ds_joysticks_name,
      int[] ds_joysticks_type,
      short[] ds_joysticks_axisCount,
      int[][] ds_joysticks_axisTypes,
      float[][] ds_joysticks_axisValues,
      int[] ds_joysticks_buttonCount,
      int[] ds_joysticks_buttons,
      short[] ds_joysticks_povCount,
      short[][] ds_joysticks_povValues,
      boolean[] ds_joysticks_isXbox,
      int pdp_handle,
      int pdp_channelCount,
      int pdp_type,
      int pdp_moduleId,
      long pdp_faults,
      long pdp_stickyFaults,
      double pdp_temperature,
      double pdp_voltage,
      double[] pdp_channelCurrent,
      double pdp_totalCurrent,
      double pdp_totalPower,
      double pdp_totalEnergy,
      int sys_fpgaVersion,
      int sys_fpgaRevision,
      int sys_serialNumberSize,
      int[] sys_serialNumber,
      int sys_commentsSize,
      int[] sys_comments,
      int sys_teamNumber,
      int sys_fpgaButton,
      int sys_systemActive,
      int sys_brownedOut,
      int sys_commsDisableCount,
      int sys_rslState,
      int sys_systemTimeValid,
      double sys_voltageVin,
      double sys_currentVin,
      double sys_userVoltage3v3,
      double sys_userCurrent3v3,
      int sys_userActive3v3,
      int sys_userCurrentFaults3v3,
      double sys_userVoltage5v,
      double sys_userCurrent5v,
      int sys_userActive5v,
      int sys_userCurrentFaults5v,
      double sys_userVoltage6v,
      double sys_userCurrent6v,
      int sys_userActive6v,
      int sys_userCurrentFaults6v,
      double sys_brownoutVoltage,
      double sys_cpuTemp,
      float sys_can_status_percentBusUtilization,
      long sys_can_status_busOffCount,
      long sys_can_status_txFullCount,
      long sys_can_status_receiveErrorCount,
      long sys_can_status_transmitErrorCount,
      long sys_epochTime) {
    builder.prep(8, 2808);
    builder.prep(8, 248);
    builder.putLong(sys_epochTime);
    builder.pad(4);
    builder.prep(4, 20);
    builder.putInt((int) sys_can_status_transmitErrorCount);
    builder.putInt((int) sys_can_status_receiveErrorCount);
    builder.putInt((int) sys_can_status_txFullCount);
    builder.putInt((int) sys_can_status_busOffCount);
    builder.putFloat(sys_can_status_percentBusUtilization);
    builder.putDouble(sys_cpuTemp);
    builder.putDouble(sys_brownoutVoltage);
    builder.putInt(sys_userCurrentFaults6v);
    builder.putInt(sys_userActive6v);
    builder.putDouble(sys_userCurrent6v);
    builder.putDouble(sys_userVoltage6v);
    builder.putInt(sys_userCurrentFaults5v);
    builder.putInt(sys_userActive5v);
    builder.putDouble(sys_userCurrent5v);
    builder.putDouble(sys_userVoltage5v);
    builder.putInt(sys_userCurrentFaults3v3);
    builder.putInt(sys_userActive3v3);
    builder.putDouble(sys_userCurrent3v3);
    builder.putDouble(sys_userVoltage3v3);
    builder.putDouble(sys_currentVin);
    builder.putDouble(sys_voltageVin);
    builder.putInt(sys_systemTimeValid);
    builder.putInt(sys_rslState);
    builder.putInt(sys_commsDisableCount);
    builder.putInt(sys_brownedOut);
    builder.putInt(sys_systemActive);
    builder.putInt(sys_fpgaButton);
    builder.putInt(sys_teamNumber);
    for (int _idx0 = 64; _idx0 > 0; _idx0--) {
      builder.putByte((byte) sys_comments[_idx0 - 1]);
    }
    builder.putShort((short) sys_commentsSize);
    for (int _idx0 = 8; _idx0 > 0; _idx0--) {
      builder.putByte((byte) sys_serialNumber[_idx0 - 1]);
    }
    builder.putShort((short) sys_serialNumberSize);
    builder.putInt(sys_fpgaRevision);
    builder.putInt(sys_fpgaVersion);
    builder.prep(8, 256);
    builder.putDouble(pdp_totalEnergy);
    builder.putDouble(pdp_totalPower);
    builder.putDouble(pdp_totalCurrent);
    for (int _idx0 = 24; _idx0 > 0; _idx0--) {
      builder.putDouble(pdp_channelCurrent[_idx0 - 1]);
    }
    builder.putDouble(pdp_voltage);
    builder.putDouble(pdp_temperature);
    builder.putInt((int) pdp_stickyFaults);
    builder.putInt((int) pdp_faults);
    builder.putInt(pdp_moduleId);
    builder.putInt(pdp_type);
    builder.putInt(pdp_channelCount);
    builder.putInt(pdp_handle);
    builder.prep(8, 2296);
    for (int _idx0 = 6; _idx0 > 0; _idx0--) {
      builder.prep(4, 356);
      builder.pad(1);
      builder.putBoolean(ds_joysticks_isXbox[_idx0 - 1]);
      for (int _idx1 = 12; _idx1 > 0; _idx1--) {
        builder.putShort(ds_joysticks_povValues[_idx0 - 1][_idx1 - 1]);
      }
      builder.putShort(ds_joysticks_povCount[_idx0 - 1]);
      builder.putInt(ds_joysticks_buttons[_idx0 - 1]);
      builder.pad(3);
      builder.putByte((byte) ds_joysticks_buttonCount[_idx0 - 1]);
      for (int _idx1 = 12; _idx1 > 0; _idx1--) {
        builder.putFloat(ds_joysticks_axisValues[_idx0 - 1][_idx1 - 1]);
      }
      for (int _idx1 = 12; _idx1 > 0; _idx1--) {
        builder.putByte((byte) ds_joysticks_axisTypes[_idx0 - 1][_idx1 - 1]);
      }
      builder.putShort(ds_joysticks_axisCount[_idx0 - 1]);
      builder.pad(1);
      builder.putByte((byte) ds_joysticks_type[_idx0 - 1]);
      for (int _idx1 = 256; _idx1 > 0; _idx1--) {
        builder.putByte((byte) ds_joysticks_name[_idx0 - 1][_idx1 - 1]);
      }
    }
    builder.putDouble(ds_matchTime);
    builder.pad(4);
    builder.putInt(ds_controlWord);
    builder.putInt(ds_matchType);
    builder.pad(3);
    builder.putByte((byte) ds_replayNumber);
    builder.putShort((short) ds_matchNumber);
    for (int _idx0 = 64; _idx0 > 0; _idx0--) {
      builder.putByte((byte) ds_gameSpecificMessage[_idx0 - 1]);
    }
    builder.putShort((short) ds_gameSpecificMessageSize);
    for (int _idx0 = 64; _idx0 > 0; _idx0--) {
      builder.putByte((byte) ds_eventName[_idx0 - 1]);
    }
    builder.putInt(ds_allianceStation);
    builder.putLong(timestamp);
    return builder.offset();
  }

  public static final class Vector extends BaseVector {
    public Vector __assign(int _vector, int _element_size, ByteBuffer _bb) {
      __reset(_vector, _element_size, _bb);
      return this;
    }

    public CoreInputs get(int j) {
      return get(new CoreInputs(), j);
    }

    public CoreInputs get(CoreInputs obj, int j) {
      return obj.__assign(__element(j), bb);
    }
  }
}
