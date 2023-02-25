// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.DataLog;
import frc.robot.lib.Enums.RobotMode;
import frc.robot.lib.Telemetry;

public class Robot extends TimedRobot {
  private static Robot m_robotInstance;
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    m_robotInstance = this;
    DataLog.start();
    Telemetry.start(); 
    m_robotContainer = new RobotContainer();    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    DataLog.mode(RobotMode.DISABLED);
    m_robotContainer.robotShouldReset();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    DataLog.mode(RobotMode.AUTO);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //TODO FIX 
    //m_robotContainer.resetRobot();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    DataLog.mode(RobotMode.TELEOP);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if(!isCompetitionMode()) {
      m_robotContainer.resetRobot();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    DataLog.mode(RobotMode.TEST);
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.resetRobot();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /** This function provides static access to create a custom periodic function in the current robot instance. */
  public static void addCustomPeriodic(Runnable callback, double periodSeconds) {
    m_robotInstance.addPeriodic(callback, periodSeconds, 0.333);
  }
  
  public static boolean isCompetitionMode() {
    // In Practice mode and in a real competition getMatchTime() returns time left in this
    // part of the match.  Otherwise it just returns -1.0.
    return DriverStation.getMatchTime() != -1;
  }
}
