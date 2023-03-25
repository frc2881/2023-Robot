// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.drive.ZeroHeadingToAng;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Suction;

public class AutoScoreMove extends SequentialCommandGroup {
  public AutoScoreMove(
    Drive drive, 
    Suction suction, 
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    Intake intake, 
    PathPlannerTrajectory trajectory
  ) {
    addCommands(
      new ZeroHeadingToAng(drive, 180),
      new AutoScore(suction, armExtension, armTilt, intake),
      new ParallelCommandGroup(
        new ResetArm(armExtension, armTilt, 1.0),
        new FollowTrajectory(trajectory, true, drive)
      )
    );
  }
}
