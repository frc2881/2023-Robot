// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Log;
import frc.robot.utils.Telemetry;

public class Robot extends TimedRobot {
  private static Robot m_robotInstance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    m_robotInstance = this;
    setupLogging();
    Telemetry.start(); 
    m_robotContainer = new RobotContainer();    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Log.mode("DISABLED");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Log.mode("AUTONOMOUS");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Log.mode("TELEOP");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

   /** This function provides static access to create a custom periodic function in the current robot instance. */
   public static void addCustomPeriodic(Runnable callback, double periodSeconds) {
    m_robotInstance.addPeriodic(callback, periodSeconds, 0.333);
  }

  private void setupLogging() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    CommandScheduler.getInstance().
      onCommandInitialize(command -> Log.init(command));
    CommandScheduler.getInstance().
      onCommandInterrupt(command -> Log.end(command, true));
    CommandScheduler.getInstance().
      onCommandFinish(command -> Log.end(command, false));

    Log.start();
  }
  
}
