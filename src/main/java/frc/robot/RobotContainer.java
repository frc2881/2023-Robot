// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmExtendOverride;
import frc.robot.commands.arm.ArmTiltOverride;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.arm.TiltArm;
import frc.robot.commands.arm.MoveTo.MoveToHigh;
import frc.robot.commands.arm.MoveTo.MoveToLow;
import frc.robot.commands.arm.MoveTo.MoveToMedium;
import frc.robot.commands.arm.MoveTo.MoveToPickup;
import frc.robot.commands.arm.Score.ScoreHigh;
import frc.robot.commands.arm.Score.ScoreMedium;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoMiddleScoreMove;
import frc.robot.commands.auto.AutoMove;
import frc.robot.commands.auto.AutoScore;
import frc.robot.commands.auto.AutoScoreBalance;
import frc.robot.commands.auto.AutoScoreMove;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.commands.controllers.RumbleControllers;
import frc.robot.commands.controllers.RumbleControllers.RumblePattern;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.ResetSwerve;
import frc.robot.commands.drive.ToggleX;
import frc.robot.commands.drive.ZeroHeading;
import frc.robot.commands.suction.ToggleSuction;
import frc.robot.lib.Utils;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PrettyLights;
import frc.robot.subsystems.PrettyLights.PanelLocation;
import frc.robot.subsystems.PrettyLights.Pattern;
import frc.robot.subsystems.Suction;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private Drive m_drive = new Drive();
  private Suction m_suction = new Suction();
  private ArmExtension m_armExtension = new ArmExtension();
  private ArmTilt m_armTilt = new ArmTilt();
  private PrettyLights m_lights = new PrettyLights();

  private final XboxController m_driverController = new XboxController(Constants.Controllers.kDriverControllerPort);
  private final XboxController m_manipulatorController = new XboxController(Constants.Controllers.kManipulatorControllerPort);

  private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();

  public boolean m_isTesting = true;
 
  public RobotContainer() {
    setupDrive(); 
    setupControllers();
    setupAuto();
    setupLights();
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
    m_drive.resetSwerve();
  }

  private void setupControllers() {
    
    // DRIVER
    new Trigger(() -> Math.abs(m_driverController.getRightTriggerAxis()) > 0.9)
      .whileTrue(new DriveRobotCentric(m_drive));

    new Trigger(m_driverController::getBackButton)
      .onTrue(new ZeroHeading(m_drive));

    new Trigger(m_driverController::getStartButton)
      .onTrue(new ResetSwerve(m_drive));

    new Trigger(m_driverController::getAButton)
      .whileTrue(new Balance(m_drive, true));

    new Trigger(m_driverController::getBButton)
      .whileTrue(new Balance(m_drive, false));
    
    new Trigger(m_driverController::getXButton)
      .onTrue(new ToggleX(m_drive));

    // new Trigger(m_driverController::getAButton)
    //   .whileTrue(new RunRollersInward(m_intake));

    // new Trigger(m_driverController::getBButton)
    //   .whileTrue(new RunRollersOutward(m_intake));

    // MANIPULATOR

    /* Toggles Suction on or off */
    new Trigger(m_manipulatorController::getAButton)
      .onTrue(new ToggleSuction(m_suction));

    new Trigger(m_manipulatorController::getXButton)
      .onTrue(new InstantCommand(() -> {m_lights.setPattern(Pattern.Heart, PanelLocation.Both);}));

    new Trigger(m_manipulatorController::getBButton)
      .onTrue(new InstantCommand(() -> {m_lights.setPattern(Pattern.None, PanelLocation.Both);}));

    new Trigger(m_manipulatorController::getLeftBumper)
      .onTrue(new InstantCommand(() -> {m_lights.setPattern(Pattern.Cube, PanelLocation.Both);}));

    new Trigger(m_manipulatorController::getRightBumper)
      .onTrue(new InstantCommand(() -> {m_lights.setPattern(Pattern.Cone, PanelLocation.Both);}));

    /* Runs the arm using joysticks */
    new Trigger(() -> Math.abs(m_manipulatorController.getLeftY()) > 0.1)
      .whileTrue(new ExtendArm(m_armExtension, m_manipulatorController::getLeftY));

    new Trigger(() -> Math.abs(m_manipulatorController.getRightY()) > 0.1)
      .whileTrue(new TiltArm(m_armTilt, m_manipulatorController::getRightY));

    /* Overrides soft limits and zeros the arm */
    new Trigger(m_manipulatorController::getBackButton)
      .whileTrue(new ArmExtendOverride(m_armExtension));

    new Trigger(m_manipulatorController::getStartButton)
      .whileTrue(new ArmTiltOverride(m_armTilt));
    
    /* Uses D-Pad to move the arm to position */
    new Trigger(() -> m_manipulatorController.getPOV() == 0)
      .whileTrue(new MoveToHigh(m_armExtension, m_armTilt, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 90)
      .whileTrue(new MoveToMedium(m_armExtension, m_armTilt, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 180)
      .whileTrue(new MoveToLow(m_armExtension, m_armTilt, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 270)
      .whileTrue(new MoveToPickup(m_armExtension, m_armTilt, 1.0, m_suction));

    /* Uses D-Pad + Y button to score */
    new Trigger(() -> m_manipulatorController.getPOV() == 0)
      .and(m_manipulatorController::getYButton)
      .whileTrue(new ScoreHigh(m_armExtension, m_armTilt, 1.0, m_suction));

    new Trigger(() -> m_manipulatorController.getPOV() == 90)
      .and(m_manipulatorController::getYButton)
      .whileTrue(new ScoreMedium(m_armExtension, m_armTilt, 1.0, m_suction));

    // RUMBLES
    new Trigger(() -> (RobotState.isTeleop() && m_suction.hasVacuum()))
      .onTrue(new RumbleControllers(m_driverController, m_manipulatorController, RumblePattern.GOOD));

      new Trigger(() -> (RobotState.isTeleop() && m_suction.hasVacuumLost()))
      .onTrue(new RumbleControllers(m_driverController, m_manipulatorController, RumblePattern.BAD));

    
  }

  public void setupAuto() {
    
    PathPlannerTrajectory balancePath = PathPlanner.loadPath("Balance", 1.0, 1.0);
    PathPlannerTrajectory balanceMidPath = PathPlanner.loadPath("Mid Balance", 1.0, 1.0);
    PathPlannerTrajectory move1Path = PathPlanner.loadPath("Move 1", 1.5, 1.5);
    PathPlannerTrajectory moveDivider5Path = PathPlanner.loadPath("Move Divider 5", 3, 3);
    PathPlannerTrajectory moveWall5Path = PathPlanner.loadPath("Move Wall 5", 3, 3);
    PathPlannerTrajectory moveDivider6Path = PathPlanner.loadPath("Move Divider 6", 3, 3);
    PathPlannerTrajectory moveWall6Path = PathPlanner.loadPath("Move Wall 6", 3, 3);
    PathPlannerTrajectory move9Path = PathPlanner.loadPath("Move 9", 1.5, 1.5);
    PathPlannerTrajectory balance1Path = PathPlanner.loadPath("Balance 1", 2, 3);
    PathPlannerTrajectory balance5Path = PathPlanner.loadPath("Balance 5", 2, 3);
    PathPlannerTrajectory balance6Path = PathPlanner.loadPath("Balance 6", 2, 3);
    PathPlannerTrajectory balance9Path = PathPlanner.loadPath("Balance 9", 2, 3);

    PathPlannerTrajectory testPath = PathPlanner.loadPath("Test", 1.5, 1.5);

    m_autonomousChooser.setDefaultOption("None", null);

    if(m_isTesting == true) {
      m_autonomousChooser.addOption("Score Cone", 
      new AutoScore(m_suction, m_armExtension, m_armTilt, false));

      m_autonomousChooser.addOption("Score Cube", 
        new AutoScore(m_suction, m_armExtension, m_armTilt, true));

        m_autonomousChooser.addOption("1 - Move", 
        new AutoMove(m_drive, move1Path));

      m_autonomousChooser.addOption("6 - Balance",
        new AutoBalance(m_drive, balance6Path, balanceMidPath, true));

      m_autonomousChooser.addOption("9 - Move", 
        new AutoMove(m_drive, move9Path));

      m_autonomousChooser.addOption("Test", 
        new FollowTrajectory(testPath, false, m_drive));
    }

    // Position 1
    m_autonomousChooser.addOption("1 - Score Move", 
      new AutoScoreMove(m_drive, m_suction, m_armExtension, m_armTilt, move1Path, false));

    m_autonomousChooser.addOption("1 - Score Balance", 
      new AutoScoreBalance(m_drive, m_suction, m_armExtension, m_armTilt, balance1Path, balancePath, false, false));

    // Position 5
    m_autonomousChooser.addOption("5 - Score Wait Move Divider",
      new AutoMiddleScoreMove(m_drive, m_suction, m_armExtension, m_armTilt, moveDivider5Path, true));

    m_autonomousChooser.addOption("5 - Score Wait Move Wall",
      new AutoMiddleScoreMove(m_drive, m_suction, m_armExtension, m_armTilt, moveWall5Path, true));

    m_autonomousChooser.addOption("5 - Score Balance",
      new AutoScoreBalance(m_drive, m_suction, m_armExtension, m_armTilt, balance5Path, balanceMidPath, true, true));

    // Position 6
    m_autonomousChooser.addOption("6 - Score Wait Move Divider",
      new AutoMiddleScoreMove(m_drive, m_suction, m_armExtension, m_armTilt, moveDivider6Path, false));

    m_autonomousChooser.addOption("6 - Score Wait Move Wall",
      new AutoMiddleScoreMove(m_drive, m_suction, m_armExtension, m_armTilt, moveWall6Path, false));
    
    m_autonomousChooser.addOption("6 - Score Balance", 
      new AutoScoreBalance(m_drive, m_suction, m_armExtension, m_armTilt, balance6Path, balanceMidPath, false, true));
    
    // Position 9
    m_autonomousChooser.addOption("9 - Score Move", 
      new AutoScoreMove(m_drive, m_suction, m_armExtension, m_armTilt, move9Path, false));

    m_autonomousChooser.addOption("9 - Score Balance", 
      new AutoScoreBalance(m_drive, m_suction, m_armExtension, m_armTilt, balance9Path, balancePath, false, false));


    SmartDashboard.putData("Auto/Command", m_autonomousChooser);
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }

  private void setupLights() {
    m_lights.setPattern(Pattern.Heart, PanelLocation.Both);

    new Trigger(() -> (DriverStation.getMatchTime() <= 35))
      .onTrue(new InstantCommand(() -> { m_lights.setPattern(Pattern.Charge, PanelLocation.Both); }));    }
  
  public void resetRobot() {
      m_drive.resetSwerve();
      m_drive.resetPhotonCameras();
      m_suction.reset();
      m_armTilt.reset();
      m_armExtension.reset();
      
  }

  public void resetLights(){
    m_lights.setPattern(Pattern.Heart, PanelLocation.Both); 
  }
}
