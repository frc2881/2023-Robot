// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory (PathPlannerTrajectory trajectory, boolean isFirstPath, Drive drive) {
        addCommands(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if(isFirstPath){
                    drive.resetOdometry(new Pose2d());
                }
              }),
            new PPSwerveControllerCommand(
                trajectory, 
                drive::getPose, 
                Constants.Drive.kDriveKinematics, 
                new PIDController(Constants.Trajectory.kTrajectoryXYP, Constants.Trajectory.kTrajectoryXYI, Constants.Trajectory.kTrajectoryXYD),
                new PIDController(Constants.Trajectory.kTrajectoryXYP, Constants.Trajectory.kTrajectoryXYI, Constants.Trajectory.kTrajectoryXYD),
                new PIDController(Constants.Trajectory.kTrajectoryRP, Constants.Trajectory.kTrajectoryRI, Constants.Trajectory.kTrajectoryRD),
                drive::setModuleStates, 
                true, 
                drive
                )
        );
    }
}
