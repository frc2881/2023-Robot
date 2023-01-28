// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.suction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Suction;

public class ReleaseSuction extends CommandBase {
  private Suction m_suction;
  /** Creates a new ReleaseSuction. */
  public ReleaseSuction(Suction suction) {
    this.m_suction = suction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_suction.release();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
