// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ResetArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class AutoDriveFlat extends SequentialCommandGroup {
  public AutoDriveFlat(Drive drive, Arm arm, Intake intake, PathPlannerTrajectory trajectory) {
    addCommands(
      Commands.parallel(
        new ResetArm(arm, intake, 0.15),
        new FollowTrajectory(trajectory, true, drive)
      )
    );
  }
}
