/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.statemachine;

import com.team1165.robot.util.logging.LoggedTunableNumber;
import java.util.EnumMap;

public class StateUtils {
  @SafeVarargs
  public static <S extends Enum<S> & State> EnumMap<S, LoggedTunableNumber> createTunableNumberMap(
      String key, S... states) {
    if (states.length != 0) {
      EnumMap<S, LoggedTunableNumber> map = new EnumMap<>(states[0].getDeclaringClass());
      for (S state : states) {
        var stateValue = state.get();
        if (stateValue.isPresent()) {
          map.put(
              state, new LoggedTunableNumber(key + "/" + state.name(), stateValue.getAsDouble()));
        }
      }
      return map;
    } else {
      throw new IllegalArgumentException("Must provide at least one state to create a map.");
    }
  }

  public static <S extends Enum<S> & State> EnumMap<S, LoggedTunableNumber> createTunableNumberMap(
      String key, Class<S> stateEnum) {
    EnumMap<S, LoggedTunableNumber> map = new EnumMap<>(stateEnum);
    for (S state : stateEnum.getEnumConstants()) {
      var stateValue = state.get();
      if (stateValue.isPresent()) {
        map.put(state, new LoggedTunableNumber(key + "/" + state.name(), stateValue.getAsDouble()));
      }
    }
    return map;
  }
}
