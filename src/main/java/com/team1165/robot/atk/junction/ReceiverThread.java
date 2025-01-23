/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.atk.junction;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class ReceiverThread extends Thread {

  private final BlockingQueue<com.team1165.robot.atk.junction.LogTable> queue;
  private List<LogDataReceiver> dataReceivers = new ArrayList<>();

  ReceiverThread(BlockingQueue<com.team1165.robot.atk.junction.LogTable> queue) {
    super("AdvantageKit_LogReceiver");
    this.setDaemon(true);
    this.queue = queue;
  }

  void addDataReceiver(LogDataReceiver dataReceiver) {
    dataReceivers.add(dataReceiver);
  }

  public void run() {
    // Start data receivers
    for (int i = 0; i < dataReceivers.size(); i++) {
      dataReceivers.get(i).start();
    }

    try {
      while (true) {
        LogTable entry = queue.take(); // Wait for data

        // Send data to receivers
        for (int i = 0; i < dataReceivers.size(); i++) {
          dataReceivers.get(i).putTable(entry);
        }
      }
    } catch (InterruptedException exception) {

      // End all data receivers
      for (int i = 0; i < dataReceivers.size(); i++) {
        dataReceivers.get(i).end();
      }
    }
  }
}
