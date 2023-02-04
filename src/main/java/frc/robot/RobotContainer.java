// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.ZeroHeading;
import frc.robot.commands.suction.DisableSuction;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Suction;
import frc.robot.subsystems.Arm;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.arm.RetractArm;
import frc.robot.utils.Log;

public class RobotContainer {
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_manipulatorController = new XboxController(1);
  private final PathPlannerTrajectory simplePath = PathPlanner.loadPath("SimplePath", 1, 1);
  Drive m_drive = new Drive();
  Suction m_suction = new Suction();
  Arm m_arm = new Arm();

  private final DriveWithJoysticks m_driveWithJoysticks = new DriveWithJoysticks(
      m_drive,
      () -> applyDeadband(-m_driverController.getLeftY()),
      () -> applyDeadband(-m_driverController.getLeftX()),
      () -> applyDeadband(-m_driverController.getRightX()));

  public RobotContainer() {
    configureControllers();

    m_drive.setDefaultCommand(m_driveWithJoysticks);

    CommandScheduler.getInstance().
      onCommandInitialize(command -> Log.init(command));
    CommandScheduler.getInstance().
      onCommandInterrupt(command -> Log.end(command, true));
    CommandScheduler.getInstance().
      onCommandFinish(command -> Log.end(command, false));

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public double applyDeadband(double input) {
    if (Math.abs(input) < 0.1) {
      return 0.0;
    } else {
      return Math.copySign((Math.abs(input) - 0.1) / 0.9, input);
    }
  }

  private void configureControllers() {

    //DRIVER
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
      .onTrue(new ZeroHeading(m_drive));

    //MANIPULATOR
    new JoystickButton(m_manipulatorController, XboxController.Button.kA.value)
      .onTrue(new EnableSuction(m_suction));

    new JoystickButton(m_manipulatorController, XboxController.Button.kB.value)
      .onTrue(new DisableSuction(m_suction));

    new JoystickButton(m_manipulatorController, XboxController.Button.kY.value)
      .whileTrue(new ExtendArm(m_arm, 0));

    new JoystickButton(m_manipulatorController, XboxController.Button.kX.value)
      .whileTrue(new RetractArm(m_arm, 0));
  }

  public Command getAutonomousCommand() {
    //return new DriveWithJoysticks(m_drive, () -> -1.0/Constants.Drive.kMaxSpeedMetersPerSecond, () -> 0, () -> 0);
    return new FollowTrajectory(simplePath, true, m_drive);
  }
}
