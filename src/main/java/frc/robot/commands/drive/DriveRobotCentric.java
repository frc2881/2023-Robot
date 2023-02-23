// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Enums.SwerveDriveMode;
import frc.robot.subsystems.Drive;

public class DriveRobotCentric extends CommandBase {
  private final Drive m_drive;

  /** Creates a new ZeroHeading. */
  public DriveRobotCentric(Drive drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setDriveMode(SwerveDriveMode.ROBOT_CENTRIC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setDriveMode(SwerveDriveMode.FIELD_CENTRIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
