// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmExtendOverride;
import frc.robot.commands.arm.ArmTiltOverride;
import frc.robot.commands.arm.ExtendAndTiltArm;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.arm.ScoreMedium;
import frc.robot.commands.arm.TiltArm;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.ZeroHeading;
import frc.robot.commands.intake.ExtendIntakeArm;
import frc.robot.commands.intake.RetractIntakeArm;
import frc.robot.commands.suction.DisableSuction;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.lib.Utils;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Suction;

public class RobotContainer {
  private Drive m_drive = new Drive();
  private Suction m_suction = new Suction();
  private Arm m_arm = new Arm();
  private Intake m_intake = new Intake();

  private final XboxController m_driverController = new XboxController(Constants.Controllers.kDriverControllerPort);
  private final XboxController m_manipulatorController = new XboxController(Constants.Controllers.kManipulatorControllerPort);

  private final PathPlannerTrajectory simplePath = PathPlanner.loadPath("SimplePath", 1, 1);
  
  public RobotContainer() {
    setupDrive(); 
    setupTriggers();
  }

  private void setupDrive() {
    m_drive.setDefaultCommand(
      new DriveWithJoysticks(
        m_drive,
        () -> Utils.applyDeadband(-m_driverController.getLeftY(), Constants.Controllers.kDeadband),
        () -> Utils.applyDeadband(-m_driverController.getLeftX(), Constants.Controllers.kDeadband),
        () -> Utils.applyDeadband(-m_driverController.getRightX(), Constants.Controllers.kDeadband)
      )
    );
  }

  private void setupTriggers() {
    //DRIVER
    new Trigger(m_driverController::getBackButton).onTrue(new ZeroHeading(m_drive));
    //new Trigger(m_driverController::getAButton).whileTrue(new RunRollersForward(m_intake));
    //new Trigger(m_driverController::getBButton).whileTrue(new RunRollersBackward(m_intake));
    new Trigger(m_driverController::getXButton).onTrue(new ExtendIntakeArm(m_intake)); 
    new Trigger(m_driverController::getYButton).onTrue(new RetractIntakeArm(m_intake)); 

    //MANIPULATOR
    new Trigger(m_manipulatorController::getAButton).onTrue(new EnableSuction(m_suction));
    new Trigger(m_manipulatorController::getBButton).onTrue(new DisableSuction(m_suction));
    new Trigger(() -> Math.abs(m_manipulatorController.getLeftY()) > 0.1).whileTrue(new ExtendArm(m_arm, m_manipulatorController::getLeftY));
    new Trigger(() -> Math.abs(m_manipulatorController.getRightY()) > 0.1).whileTrue(new TiltArm(m_arm, m_manipulatorController::getRightY)); 
    new Trigger(m_manipulatorController::getXButton).whileTrue(new ExtendAndTiltArm(0.2, 8.0, 8.0, m_arm, m_intake));
    new Trigger(m_manipulatorController::getBackButton).whileTrue(new ArmExtendOverride(m_arm));
    new Trigger(m_manipulatorController::getStartButton).whileTrue(new ArmTiltOverride(m_arm));
    new Trigger(m_manipulatorController::getYButton).whileTrue(new ScoreMedium(m_arm, m_intake, 0.15, m_suction));

    new Trigger(() -> m_manipulatorController.getPOV() == 0).whileTrue(new RunCommand(() -> m_arm.setDesiredExtensionPosition(10), m_arm));
    new Trigger(() -> m_manipulatorController.getPOV() == 180).whileTrue(new RunCommand(() -> m_arm.setDesiredExtensionPosition(2), m_arm));
    new Trigger(() -> m_manipulatorController.getPOV() == 90).whileTrue(new RunCommand(() -> m_arm.setDesiredTiltPosition(10), m_arm));
    new Trigger(() -> m_manipulatorController.getPOV() == 270).whileTrue(new RunCommand(() -> m_arm.setDesiredTiltPosition(2), m_arm));
    
  }

  public Command getAutonomousCommand() {
    return new RunCommand(() -> m_drive.drive(1.0, 0.0, 0.0, true), m_drive); // DriveWithJoysticks(m_drive, () -> 1.0, () -> 0.0, () -> 0.0); // FollowTrajectory(simplePath, true, m_drive);
  }
}
