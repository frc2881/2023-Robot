// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.commands.arm.ExtendArm;
// import frc.robot.commands.arm.RetractArm;
// import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.ZeroHeading;
// import frc.robot.commands.suction.DisableSuction;
// import frc.robot.commands.suction.EnableSuction;
import frc.robot.lib.Utils;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Suction;

public class RobotContainer {
  private Swerve m_swerve = new Swerve();
  // private Suction m_suction = new Suction();
  // private Arm m_arm = new Arm();
  private Elevator m_elevator = new Elevator();

  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Controllers.kDriverControllerPort);
  private final CommandXboxController m_manipulatorController = new CommandXboxController(Constants.Controllers.kManipulatorControllerPort);

  private final PathPlannerTrajectory simplePath = PathPlanner.loadPath("SimplePath", 1, 1);
  
  public RobotContainer() {
    m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.motorsOff(), m_elevator));
    setupDrive(); 
    configureButtonBindings();
  }

  private void setupDrive() {
    m_swerve.setDefaultCommand(
      new DriveWithJoysticks(
        m_swerve,
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), Constants.Controllers.kDeadband),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), Constants.Controllers.kDeadband),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), Constants.Controllers.kDeadband)
      )
    );
  }

  private void configureButtonBindings() {
    //DRIVER
    m_driverController.back().onTrue(new ZeroHeading(m_swerve));

    //MANIPULATOR
    // new Trigger(m_manipulatorController::getAButton).onTrue(new EnableSuction(m_suction));
    // new Trigger(m_manipulatorController::getBButton).onTrue(new DisableSuction(m_suction));
    // new Trigger(m_manipulatorController::getYButton).whileTrue(new ExtendArm(m_arm, 0));
    // new Trigger(m_manipulatorController::getXButton).whileTrue(new RetractArm(m_arm, 0));

    //Elevator
    m_driverController.povUp().whileTrue(new RunCommand(() -> m_elevator.motorsOn(0.5), m_elevator)).or(m_driverController.povDown().whileTrue(new RunCommand(() -> m_elevator.motorsOn(-0.5), m_elevator)))
    .whileFalse(new RunCommand(m_elevator::motorsOff, m_elevator));
  }

  // public Command getAutonomousCommand() {
  //    return new FollowTrajectory(simplePath, true, m_drive);
  // }

  public void disabledInit()
  {
    m_swerve.setDriveMotorIdleMode(IdleMode.kCoast);
    m_swerve.setTurnMotorIdleMode(IdleMode.kCoast);
  }

  public void enabledInit()
  {
    m_swerve.setDriveMotorIdleMode(IdleMode.kBrake);
    m_swerve.setTurnMotorIdleMode(IdleMode.kBrake);
  }
}
