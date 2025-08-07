/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.logging;

import com.ctre.phoenix6.configs.Slot0Configs;

public class LoggedTunablePID {
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kG;

  public LoggedTunablePID(String key, Slot0Configs baseGains) {
    kP = new LoggedTunableNumber(key + "/kP", baseGains.kP);
    kI = new LoggedTunableNumber(key + "/kI", baseGains.kI);
    kD = new LoggedTunableNumber(key + "/kD", baseGains.kD);
    kS = new LoggedTunableNumber(key + "/kS", baseGains.kS);
    kV = new LoggedTunableNumber(key + "/kV", baseGains.kV);
    kA = new LoggedTunableNumber(key + "/kA", baseGains.kA);
    kG = new LoggedTunableNumber(key + "/kG", baseGains.kG);
  }

  public boolean hasChanged() {
    return kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kG.hasChanged(hashCode());
  }

  public double kP() {
    return kP.get();
  }

  public double kI() {
    return kI.get();
  }

  public double kD() {
    return kD.get();
  }

  public double kS() {
    return kS.get();
  }

  public double kV() {
    return kV.get();
  }

  public double kA() {
    return kA.get();
  }

  public double kG() {
    return kG.get();
  }

  public Slot0Configs toSlot0Configs() {
    return new Slot0Configs()
        .withKP(kP())
        .withKI(kI())
        .withKD(kD())
        .withKS(kS())
        .withKV(kV())
        .withKA(kA())
        .withKG(kG());
  }
}
