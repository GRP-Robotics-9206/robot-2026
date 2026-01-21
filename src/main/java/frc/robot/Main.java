// Based on original work by Littleton Robotics (2021-2026)
// Source: http://github.com/Mechanical-Advantage
// 
// Modified by GRP Robotics (2026)
// Source: https://github.com/GRP-Robotics-9206
//
// -------------------------------------------------------------------------
// Original license notice:
// Copyright (c) 2021-2026 Littleton Robotics
// Use of this source code is governed by a BSD license that can be found 
// in the LICENSE file at the root directory of this project.
// -------------------------------------------------------------------------

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
