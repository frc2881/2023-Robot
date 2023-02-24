// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.suction;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Suction;

public class ToggleSuction extends CommandBase {
  private Suction m_suction;

  public ToggleSuction(Suction suction) {
    m_suction = suction;
    addRequirements(m_suction);
  }

  @Override
  public void initialize() {
    m_suction.toggle();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
