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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmExtendOverride;
import frc.robot.commands.arm.ArmTiltOverride;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.arm.TiltArm;
import frc.robot.commands.arm.MoveTo.MoveToLow;
import frc.robot.commands.arm.MoveTo.MoveToPickupSubstation;
import frc.robot.commands.arm.Score.ScoreHigh;
import frc.robot.commands.arm.Score.ScoreHighCone;
import frc.robot.commands.arm.Score.ScoreHighCube;
import frc.robot.commands.arm.Score.ScoreMedium;
import frc.robot.commands.arm.Score.ScoreMediumCone;
import frc.robot.commands.arm.Score.ScoreMediumCube;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.commands.auto.Move;
import frc.robot.commands.auto.MoveToBalance;
import frc.robot.commands.auto.ScoreCone;
import frc.robot.commands.auto.ScoreConeMove;
import frc.robot.commands.auto.ScoreConeMovePickup;
import frc.robot.commands.auto.ScoreConeMoveToBalance;
import frc.robot.commands.auto.ScoreConeWaitMove;
import frc.robot.commands.auto.ScoreCube;
import frc.robot.commands.auto.ScoreCubeMoveToBalance;
import frc.robot.commands.auto.ScoreCubeWaitMove;
import frc.robot.commands.controllers.RumbleControllers;
import frc.robot.commands.controllers.RumbleControllers.RumblePattern;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.ResetSwerve;
import frc.robot.commands.drive.ToggleXConfiguration;
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
    
    // DRIVER CONTROLLER =========================

    new Trigger(() -> Math.abs(m_driverController.getRightTriggerAxis()) > 0.9)
      .whileTrue(new DriveRobotCentric(m_drive));

    new Trigger(m_driverController::getBackButton)
      .onTrue(new ZeroHeading(m_drive));

    new Trigger(m_driverController::getStartButton)
      .onTrue(new ResetSwerve(m_drive));

    // new Trigger(m_driverController::getAButton)
    //   .onTrue(new AlignToNearestNode(m_drive, m_drive::getTrajectoryForNearestNode));

    new Trigger(m_driverController::getBButton)
      .whileTrue(new AutoBalance(m_drive, false));
    
    new Trigger(m_driverController::getXButton)
      .onTrue(new ToggleXConfiguration(m_drive));

    // MANIPULATOR CONTROLLER =========================

    /* Toggles Suction on or off */
    new Trigger(m_manipulatorController::getAButton)
      .onTrue(new ToggleSuction(m_suction));

    // new Trigger(m_manipulatorController::getYButton)
    //   .onTrue(new MoveToPickupFloor(m_armTilt, m_armExtension, m_suction));

    /* Changes lights to different patterns */
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
      .and(() -> m_manipulatorController.getRightTriggerAxis() < 0.1)
      .and(() -> m_manipulatorController.getLeftTriggerAxis() < 0.1)
      .whileTrue(new ScoreHigh(m_armExtension, m_armTilt, m_drive, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 90)
      .and(() -> m_manipulatorController.getRightTriggerAxis() < 0.1)
      .and(() -> m_manipulatorController.getLeftTriggerAxis() < 0.1)
      .whileTrue(new ScoreMedium(m_armExtension, m_armTilt, m_drive, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 180)
      .whileTrue(new MoveToLow(m_armExtension, m_armTilt, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 270)
      .whileTrue(new MoveToPickupSubstation(m_armExtension, m_armTilt, 1.0, m_suction));


    /* Scoring Manual Override */
    new Trigger(() -> m_manipulatorController.getPOV() == 0)
      .and(() -> m_manipulatorController.getRightTriggerAxis() > 0.1)
      .whileTrue(new ScoreHighCone(m_armExtension, m_armTilt, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 0)
      .and(() -> m_manipulatorController.getLeftTriggerAxis() > 0.1)
      .whileTrue(new ScoreHighCube(m_armExtension, m_armTilt, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 90)
      .and(() -> m_manipulatorController.getRightTriggerAxis() > 0.1)
      .whileTrue(new ScoreMediumCone(m_armExtension, m_armTilt, 1.0));

    new Trigger(() -> m_manipulatorController.getPOV() == 90)
      .and(() -> m_manipulatorController.getLeftTriggerAxis() > 0.1)
      .whileTrue(new ScoreMediumCube(m_armExtension, m_armTilt, 1.0));

    // RUMBLES ============================================

    new Trigger(() -> (RobotState.isTeleop() && (m_suction.isVacuumEnabledForCone() || m_suction.isVacuumEnabledForCube())))
      .onTrue(new RumbleControllers(m_driverController, m_manipulatorController, RumblePattern.GOOD));

    new Trigger(() -> (RobotState.isTeleop() && m_suction.isVacuumDisabled()))
      .onTrue(new RumbleControllers(m_driverController, m_manipulatorController, RumblePattern.BAD));
  }

  public void setupAuto() {
    
    PathPlannerTrajectory move1Path = PathPlanner.loadPath("Move 1", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    PathPlannerTrajectory moveDivider5Path = PathPlanner.loadPath("Move Divider 5", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    PathPlannerTrajectory moveWall5Path = PathPlanner.loadPath("Move Wall 5", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    PathPlannerTrajectory moveDivider6Path = PathPlanner.loadPath("Move Divider 6", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    PathPlannerTrajectory moveWall6Path = PathPlanner.loadPath("Move Wall 6", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    PathPlannerTrajectory move9Path = PathPlanner.loadPath("Move 9", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    
    PathPlannerTrajectory moveToCone9Path = PathPlanner.loadPath("Move To Cone 9", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    PathPlannerTrajectory return9Path = PathPlanner.loadPath("Return 9", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    
    
    PathPlannerTrajectory moveToBalance1Path = PathPlanner.loadPath("Balance 1", Constants.Autonomous.kMoveToBalanceMaxVelocity, Constants.Autonomous.kMoveToBalanceMaxAccel);
    PathPlannerTrajectory moveToBalance5Path = PathPlanner.loadPath("Balance 5", Constants.Autonomous.kMoveToBalanceMaxVelocity, Constants.Autonomous.kMoveToBalanceMaxAccel);
    PathPlannerTrajectory moveToBalance6Path = PathPlanner.loadPath("Balance 6", Constants.Autonomous.kMoveToBalanceMaxVelocity, Constants.Autonomous.kMoveToBalanceMaxAccel);
    PathPlannerTrajectory moveToBalance9Path = PathPlanner.loadPath("Balance 9", Constants.Autonomous.kMoveToBalanceMaxVelocity, Constants.Autonomous.kMoveToBalanceMaxAccel);

    PathPlannerTrajectory balancePath = PathPlanner.loadPath("Balance", Constants.Autonomous.kBalanceMaxVelocity, Constants.Autonomous.kBalanceMaxAccel);
    PathPlannerTrajectory balanceMidPath = PathPlanner.loadPath("Mid Balance", Constants.Autonomous.kBalanceMaxVelocity, Constants.Autonomous.kBalanceMaxAccel);

    PathPlannerTrajectory moveToPickupPath = PathPlanner.loadPath("Move to Pickup", Constants.Autonomous.kPickupMaxVelocity, Constants.Autonomous.kPickupMaxAccel);
    

    PathPlannerTrajectory testPath = PathPlanner.loadPath("Test", 1.5, 1.5);

    m_autonomousChooser.setDefaultOption("None", null);

    // Position 1 - Cone
    m_autonomousChooser.addOption("1 - Score Move", 
      new ScoreConeMove(m_drive, m_suction, m_armExtension, m_armTilt, move1Path));

    m_autonomousChooser.addOption("1 - Score Balance", 
      new ScoreConeMoveToBalance(m_drive, m_suction, m_armExtension, m_armTilt, moveToBalance1Path, balancePath, false));

    // Position 5 - Cube
    m_autonomousChooser.addOption("5 - Score Wait Move Divider",
      new ScoreCubeWaitMove(m_drive, m_suction, m_armExtension, m_armTilt, moveDivider5Path));

    m_autonomousChooser.addOption("5 - Score Wait Move Wall",
      new ScoreCubeWaitMove(m_drive, m_suction, m_armExtension, m_armTilt, moveWall5Path));

    m_autonomousChooser.addOption("5 - Score Balance",
      new ScoreCubeMoveToBalance(m_drive, m_suction, m_armExtension, m_armTilt, moveToBalance5Path, balanceMidPath, true));

    // Position 6 - Cone
    m_autonomousChooser.addOption("6 - Score Wait Move Divider",
      new ScoreConeWaitMove(m_drive, m_suction, m_armExtension, m_armTilt, moveDivider6Path));

    m_autonomousChooser.addOption("6 - Score Wait Move Wall",
      new ScoreConeWaitMove(m_drive, m_suction, m_armExtension, m_armTilt, moveWall6Path));
    
    m_autonomousChooser.addOption("6 - Score Balance", 
      new ScoreConeMoveToBalance(m_drive, m_suction, m_armExtension, m_armTilt, moveToBalance6Path, balanceMidPath, true));
    
    // Position 9 - Cone
    m_autonomousChooser.addOption("9 - Score Move", 
      new ScoreConeMove(m_drive, m_suction, m_armExtension, m_armTilt, move9Path));

    m_autonomousChooser.addOption("9 - Score Balance", 
      new ScoreConeMoveToBalance(m_drive, m_suction, m_armExtension, m_armTilt, moveToBalance9Path, balancePath, false));

    m_autonomousChooser.addOption("9 - Score Move Pickup", 
      new ScoreConeMovePickup(m_drive, m_suction, m_armExtension, m_armTilt, moveToCone9Path, moveToPickupPath, return9Path));
    
    // TESTING

    m_autonomousChooser.addOption("TEST: Score Cone", 
      new ScoreCone(m_suction, m_armExtension, m_armTilt));

    m_autonomousChooser.addOption("TEST: Score Cube", 
      new ScoreCube(m_suction, m_armExtension, m_armTilt));

    m_autonomousChooser.addOption("TEST: 1 - Move", 
      new Move(m_drive, move1Path));

    m_autonomousChooser.addOption("TEST: 6 - Balance",
      new MoveToBalance(m_drive, moveToBalance6Path, balanceMidPath, true));

    m_autonomousChooser.addOption("TEST: 9 - Move", 
      new Move(m_drive, move9Path));

    m_autonomousChooser.addOption("TEST: Test", 
      new FollowTrajectory(testPath, false, m_drive));


    SmartDashboard.putData("Auto/Command", m_autonomousChooser);
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }

  private void setupLights() {
    m_lights.setPattern(Pattern.Heart, PanelLocation.Both);

    new Trigger(() -> (DriverStation.getMatchTime() <= 15))
      .whileTrue(new InstantCommand(() -> { m_lights.setPattern(Pattern.Charge, PanelLocation.Both); }));  

    // STARTING CONFIGURATION ============================================

    new Trigger(() -> (RobotState.isDisabled() && Utils.isValueBetween(m_armExtension.getEncoderPosition(), 3.35, 4.0)))
      .whileTrue(Commands.runOnce(() -> { m_lights.setPattern(Pattern.Cube, PanelLocation.Both); }).ignoringDisable(true))
      .onFalse(Commands.runOnce(() -> { m_lights.setPattern(Pattern.Heart, PanelLocation.Both); }).ignoringDisable(true));

    new Trigger(() -> (RobotState.isDisabled() && Utils.isValueBetween(m_armExtension.getEncoderPosition(), 5.75, 5.85)))
      .whileTrue(Commands.runOnce(() -> { m_lights.setPattern(Pattern.Cone, PanelLocation.Both); }).ignoringDisable(true))
      .onFalse(Commands.runOnce(() -> { m_lights.setPattern(Pattern.Heart, PanelLocation.Both); }).ignoringDisable(true));
   }
  
  public void resetRobot() {
      m_drive.resetSwerve();
      m_suction.reset();
      m_armTilt.reset();
      m_armExtension.reset();
  }

  public void resetLights() {
    m_lights.setPattern(Pattern.Heart, PanelLocation.Both); 
  }
}
