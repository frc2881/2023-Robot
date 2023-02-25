// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Suction;

public class AutoSequenceA extends SequentialCommandGroup {
  public AutoSequenceA(Drive drive, Suction suction, Arm arm, Intake intake, PathPlannerTrajectory trajectory) {
    addCommands(
      new AutoScoreHigh(suction, arm, intake),
      new AutoDriveFlat(drive, arm, intake, trajectory)
    );
  }
}
