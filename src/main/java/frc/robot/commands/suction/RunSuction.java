// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.suction;

import frc.robot.subsystems.Suction;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunSuction extends CommandBase {
  private Suction m_suction;

  public RunSuction(Suction suction) {
    this.m_suction = suction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_suction.run(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_suction.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
