// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.lib.Enums.RobotMode;
import frc.robot.lib.Enums.RobotStatus;

public final class Telemetry {
  private static boolean m_isAllTelemetryEnabled = false;

  /** Initializes default telemetry settings and topics with custom periodic updates */
  public static void start() {
    SmartDashboard.putBoolean("EnableAllTelemetry", m_isAllTelemetryEnabled);

    Robot.addCustomPeriodic(Telemetry::updateRobotInfo, 0.3);
    Robot.addCustomPeriodic(Telemetry::updateMatchInfo, 0.3);
    Robot.addCustomPeriodic(Telemetry::updateFPGATimestamp, 1.0);
    Robot.addCustomPeriodic(Telemetry::updateTelemetrySetting, 3.0);
  }

  /** This periodic function supports providing the current robot mode and status to the driver station dashbaoard app */
  private static void updateRobotInfo() {
    RobotMode mode = RobotMode.DISABLED;
    if (RobotState.isAutonomous()) { mode = RobotMode.AUTO; }
    if (RobotState.isTeleop()) { mode = RobotMode.TELEOP; }
    if (RobotState.isTest()) { mode = RobotMode.TEST; }
    SmartDashboard.putString("Robot/Mode", mode.toString());

    RobotStatus status = RobotStatus.DISABLED;
    if (RobotState.isEnabled()) { status = RobotStatus.ENABLED; }
    if (RobotState.isEStopped()) { status = RobotStatus.ESTOPPED; }
    SmartDashboard.putString("Robot/Status", status.toString());
  }

  /** This periodic function supports providing the current match time to the driver station dashboard app during competition mode. */
  private static void updateMatchInfo() {
    SmartDashboard.putNumber("Timing/MatchTime", Math.floor(Timer.getMatchTime()));
  }

  /** This periodic function supports adding FPGA timestamps to entries when using the NetworkTables v3 protocol. */
  private static void updateFPGATimestamp() {
    SmartDashboard.putNumber("Timing/FPGATimestamp", Timer.getFPGATimestamp());
  }

  /** This periodic function supports enabling/disabling the full stream of telemetry pushed to LiveWindow. */
  private static void updateTelemetrySetting() {
    boolean isAllTelemetryEnabled = SmartDashboard.getBoolean("EnableAllTelemetry", m_isAllTelemetryEnabled);
    if (m_isAllTelemetryEnabled != isAllTelemetryEnabled) {
      m_isAllTelemetryEnabled = isAllTelemetryEnabled;
      if (m_isAllTelemetryEnabled) {
        LiveWindow.enableAllTelemetry();
      } else {
        LiveWindow.disableAllTelemetry();
      }
    }
  }
}
