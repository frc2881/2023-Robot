// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.commands.arm.MoveTo.MoveToPickupFloor;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.ZeroHeadingToAng;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Suction;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeMovePickup extends SequentialCommandGroup {
  /** Creates a new ScoreConeMovePickupCone. */
  public ScoreConeMovePickup(
    Drive drive, 
    Suction suction, 
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    PathPlannerTrajectory moveTrajectory,
    PathPlannerTrajectory pickUpTrajectory,
    PathPlannerTrajectory returnTrajectory
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ZeroHeadingToAng(drive, 180),
      new ScoreCone(suction, armExtension, armTilt),
      new ParallelCommandGroup(
        new ResetArm(armExtension, armTilt, 1.0),
        new FollowTrajectory(moveTrajectory, true, drive)),
      new MoveToPickupFloor(armTilt, armExtension, suction),
      new ParallelRaceGroup(
        new FollowTrajectory(pickUpTrajectory, false, drive),
        new WaitUntilCommand(suction::isVacuumEnabledForCone)),
      new ConditionalCommand(
        new SequentialCommandGroup(
          new TiltArmToHeight(armTilt, 0.5, 6.0, false).withTimeout(Constants.Arm.kTiltTimeOut)
          // new FollowTrajectory(returnTrajectory, false, drive)
          // Yeet cone?
          ), 
        new WaitCommand(0.001), 
        suction::isVacuumEnabledForCone)
        
      
      
      
    );
  }
}
