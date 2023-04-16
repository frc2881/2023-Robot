// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.SetXConfiguration;
import frc.robot.commands.drive.ZeroHeadingToAng;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Suction;

public class ScoreCubeMoveToBalance extends SequentialCommandGroup {
  public ScoreCubeMoveToBalance(
    Drive drive, 
    Suction suction, 
    ArmExtension armExtension, 
    ArmTilt armTilt,
    PathPlannerTrajectory moveTrajectory, 
    PathPlannerTrajectory balanceTrajectory,
    boolean reversed
  ) {
    addCommands(
      new ZeroHeadingToAng(drive, 180),
      new ScoreCube(suction, armExtension, armTilt),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new ResetArm(armExtension, armTilt, 1.0),
        new FollowTrajectory(moveTrajectory, true, drive)
      ),
      new FollowTrajectory(balanceTrajectory, false, drive),
      new AutoBalance2(drive, reversed),
      new SetXConfiguration(drive)
    );
  }
}
