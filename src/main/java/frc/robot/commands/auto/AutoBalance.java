// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.SetX;
import frc.robot.commands.drive.ZeroHeadingToAng;
import frc.robot.subsystems.Drive;

public class AutoBalance extends SequentialCommandGroup {

  public AutoBalance(
    Drive drive,
    PathPlannerTrajectory moveTrajectory, 
    PathPlannerTrajectory balanceTrajectory
  ) {
    addCommands(
      new ZeroHeadingToAng(drive, 180),
      new FollowTrajectory(moveTrajectory, true, drive),
      new FollowTrajectory(balanceTrajectory, false, drive),
      new Balance(drive, false),
      new SetX(drive)
    );
  }

}
