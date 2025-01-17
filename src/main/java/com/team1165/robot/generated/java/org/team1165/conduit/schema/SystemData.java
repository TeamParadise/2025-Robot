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
public final class SystemData extends Struct {
  public void __init(int _i, ByteBuffer _bb) {
    __reset(_i, _bb);
  }

  public SystemData __assign(int _i, ByteBuffer _bb) {
    __init(_i, _bb);
    return this;
  }

  public int fpgaVersion() {
    return bb.getInt(bb_pos + 0);
  }

  public void mutateFpgaVersion(int fpga_version) {
    bb.putInt(bb_pos + 0, fpga_version);
  }

  public int fpgaRevision() {
    return bb.getInt(bb_pos + 4);
  }

  public void mutateFpgaRevision(int fpga_revision) {
    bb.putInt(bb_pos + 4, fpga_revision);
  }

  public int serialNumberSize() {
    return bb.getShort(bb_pos + 8) & 0xFFFF;
  }

  public void mutateSerialNumberSize(int serial_number_size) {
    bb.putShort(bb_pos + 8, (short) serial_number_size);
  }

  public int serialNumber(int j) {
    return bb.get(bb_pos + 10 + j * 1);
  }

  public void mutateSerialNumber(int j, int serial_number) {
    bb.put(bb_pos + 10 + j * 1, (byte) serial_number);
  }

  public int commentsSize() {
    return bb.getShort(bb_pos + 18) & 0xFFFF;
  }

  public void mutateCommentsSize(int comments_size) {
    bb.putShort(bb_pos + 18, (short) comments_size);
  }

  public int comments(int j) {
    return bb.get(bb_pos + 20 + j * 1);
  }

  public void mutateComments(int j, int comments) {
    bb.put(bb_pos + 20 + j * 1, (byte) comments);
  }

  public int teamNumber() {
    return bb.getInt(bb_pos + 84);
  }

  public void mutateTeamNumber(int team_number) {
    bb.putInt(bb_pos + 84, team_number);
  }

  public int fpgaButton() {
    return bb.getInt(bb_pos + 88);
  }

  public void mutateFpgaButton(int fpga_button) {
    bb.putInt(bb_pos + 88, fpga_button);
  }

  public int systemActive() {
    return bb.getInt(bb_pos + 92);
  }

  public void mutateSystemActive(int system_active) {
    bb.putInt(bb_pos + 92, system_active);
  }

  public int brownedOut() {
    return bb.getInt(bb_pos + 96);
  }

  public void mutateBrownedOut(int browned_out) {
    bb.putInt(bb_pos + 96, browned_out);
  }

  public int commsDisableCount() {
    return bb.getInt(bb_pos + 100);
  }

  public void mutateCommsDisableCount(int comms_disable_count) {
    bb.putInt(bb_pos + 100, comms_disable_count);
  }

  public int rslState() {
    return bb.getInt(bb_pos + 104);
  }

  public void mutateRslState(int rsl_state) {
    bb.putInt(bb_pos + 104, rsl_state);
  }

  public int systemTimeValid() {
    return bb.getInt(bb_pos + 108);
  }

  public void mutateSystemTimeValid(int system_time_valid) {
    bb.putInt(bb_pos + 108, system_time_valid);
  }

  public double voltageVin() {
    return bb.getDouble(bb_pos + 112);
  }

  public void mutateVoltageVin(double voltage_vin) {
    bb.putDouble(bb_pos + 112, voltage_vin);
  }

  public double currentVin() {
    return bb.getDouble(bb_pos + 120);
  }

  public void mutateCurrentVin(double current_vin) {
    bb.putDouble(bb_pos + 120, current_vin);
  }

  public double userVoltage3v3() {
    return bb.getDouble(bb_pos + 128);
  }

  public void mutateUserVoltage3v3(double user_voltage_3v3) {
    bb.putDouble(bb_pos + 128, user_voltage_3v3);
  }

  public double userCurrent3v3() {
    return bb.getDouble(bb_pos + 136);
  }

  public void mutateUserCurrent3v3(double user_current_3v3) {
    bb.putDouble(bb_pos + 136, user_current_3v3);
  }

  public int userActive3v3() {
    return bb.getInt(bb_pos + 144);
  }

  public void mutateUserActive3v3(int user_active_3v3) {
    bb.putInt(bb_pos + 144, user_active_3v3);
  }

  public int userCurrentFaults3v3() {
    return bb.getInt(bb_pos + 148);
  }

  public void mutateUserCurrentFaults3v3(int user_current_faults_3v3) {
    bb.putInt(bb_pos + 148, user_current_faults_3v3);
  }

  public double userVoltage5v() {
    return bb.getDouble(bb_pos + 152);
  }

  public void mutateUserVoltage5v(double user_voltage_5v) {
    bb.putDouble(bb_pos + 152, user_voltage_5v);
  }

  public double userCurrent5v() {
    return bb.getDouble(bb_pos + 160);
  }

  public void mutateUserCurrent5v(double user_current_5v) {
    bb.putDouble(bb_pos + 160, user_current_5v);
  }

  public int userActive5v() {
    return bb.getInt(bb_pos + 168);
  }

  public void mutateUserActive5v(int user_active_5v) {
    bb.putInt(bb_pos + 168, user_active_5v);
  }

  public int userCurrentFaults5v() {
    return bb.getInt(bb_pos + 172);
  }

  public void mutateUserCurrentFaults5v(int user_current_faults_5v) {
    bb.putInt(bb_pos + 172, user_current_faults_5v);
  }

  public double userVoltage6v() {
    return bb.getDouble(bb_pos + 176);
  }

  public void mutateUserVoltage6v(double user_voltage_6v) {
    bb.putDouble(bb_pos + 176, user_voltage_6v);
  }

  public double userCurrent6v() {
    return bb.getDouble(bb_pos + 184);
  }

  public void mutateUserCurrent6v(double user_current_6v) {
    bb.putDouble(bb_pos + 184, user_current_6v);
  }

  public int userActive6v() {
    return bb.getInt(bb_pos + 192);
  }

  public void mutateUserActive6v(int user_active_6v) {
    bb.putInt(bb_pos + 192, user_active_6v);
  }

  public int userCurrentFaults6v() {
    return bb.getInt(bb_pos + 196);
  }

  public void mutateUserCurrentFaults6v(int user_current_faults_6v) {
    bb.putInt(bb_pos + 196, user_current_faults_6v);
  }

  public double brownoutVoltage() {
    return bb.getDouble(bb_pos + 200);
  }

  public void mutateBrownoutVoltage(double brownout_voltage) {
    bb.putDouble(bb_pos + 200, brownout_voltage);
  }

  public double cpuTemp() {
    return bb.getDouble(bb_pos + 208);
  }

  public void mutateCpuTemp(double cpu_temp) {
    bb.putDouble(bb_pos + 208, cpu_temp);
  }

  public org.littletonrobotics.conduit.schema.CANStatus canStatus() {
    return canStatus(new org.littletonrobotics.conduit.schema.CANStatus());
  }

  public org.littletonrobotics.conduit.schema.CANStatus canStatus(
      org.littletonrobotics.conduit.schema.CANStatus obj) {
    return obj.__assign(bb_pos + 216, bb);
  }

  public long epochTime() {
    return bb.getLong(bb_pos + 240);
  }

  public void mutateEpochTime(long epoch_time) {
    bb.putLong(bb_pos + 240, epoch_time);
  }

  public static int createSystemData(
      FlatBufferBuilder builder,
      int fpgaVersion,
      int fpgaRevision,
      int serialNumberSize,
      int[] serialNumber,
      int commentsSize,
      int[] comments,
      int teamNumber,
      int fpgaButton,
      int systemActive,
      int brownedOut,
      int commsDisableCount,
      int rslState,
      int systemTimeValid,
      double voltageVin,
      double currentVin,
      double userVoltage3v3,
      double userCurrent3v3,
      int userActive3v3,
      int userCurrentFaults3v3,
      double userVoltage5v,
      double userCurrent5v,
      int userActive5v,
      int userCurrentFaults5v,
      double userVoltage6v,
      double userCurrent6v,
      int userActive6v,
      int userCurrentFaults6v,
      double brownoutVoltage,
      double cpuTemp,
      float can_status_percentBusUtilization,
      long can_status_busOffCount,
      long can_status_txFullCount,
      long can_status_receiveErrorCount,
      long can_status_transmitErrorCount,
      long epochTime) {
    builder.prep(8, 248);
    builder.putLong(epochTime);
    builder.pad(4);
    builder.prep(4, 20);
    builder.putInt((int) can_status_transmitErrorCount);
    builder.putInt((int) can_status_receiveErrorCount);
    builder.putInt((int) can_status_txFullCount);
    builder.putInt((int) can_status_busOffCount);
    builder.putFloat(can_status_percentBusUtilization);
    builder.putDouble(cpuTemp);
    builder.putDouble(brownoutVoltage);
    builder.putInt(userCurrentFaults6v);
    builder.putInt(userActive6v);
    builder.putDouble(userCurrent6v);
    builder.putDouble(userVoltage6v);
    builder.putInt(userCurrentFaults5v);
    builder.putInt(userActive5v);
    builder.putDouble(userCurrent5v);
    builder.putDouble(userVoltage5v);
    builder.putInt(userCurrentFaults3v3);
    builder.putInt(userActive3v3);
    builder.putDouble(userCurrent3v3);
    builder.putDouble(userVoltage3v3);
    builder.putDouble(currentVin);
    builder.putDouble(voltageVin);
    builder.putInt(systemTimeValid);
    builder.putInt(rslState);
    builder.putInt(commsDisableCount);
    builder.putInt(brownedOut);
    builder.putInt(systemActive);
    builder.putInt(fpgaButton);
    builder.putInt(teamNumber);
    for (int _idx0 = 64; _idx0 > 0; _idx0--) {
      builder.putByte((byte) comments[_idx0 - 1]);
    }
    builder.putShort((short) commentsSize);
    for (int _idx0 = 8; _idx0 > 0; _idx0--) {
      builder.putByte((byte) serialNumber[_idx0 - 1]);
    }
    builder.putShort((short) serialNumberSize);
    builder.putInt(fpgaRevision);
    builder.putInt(fpgaVersion);
    return builder.offset();
  }

  public static final class Vector extends BaseVector {
    public Vector __assign(int _vector, int _element_size, ByteBuffer _bb) {
      __reset(_vector, _element_size, _bb);
      return this;
    }

    public SystemData get(int j) {
      return get(new SystemData(), j);
    }

    public SystemData get(SystemData obj, int j) {
      return obj.__assign(__element(j), bb);
    }
  }
}
