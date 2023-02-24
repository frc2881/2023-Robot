// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DriveWithJoysticks extends CommandBase {
  private final Drive m_drive;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  public DriveWithJoysticks(
    Drive drive,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier
  ) {
    m_drive = drive;
    addRequirements(m_drive);

    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;
    m_rotationSupplier = rotationSupplier;
  }

  @Override
  public void execute() {
    m_drive.drive(
      m_translationXSupplier.getAsDouble() * Constants.Drive.kMaxSpeedMetersPerSecond,
      m_translationYSupplier.getAsDouble() * Constants.Drive.kMaxSpeedMetersPerSecond, 
      m_rotationSupplier.getAsDouble() * Constants.Drive.kMaxAngularSpeed
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
