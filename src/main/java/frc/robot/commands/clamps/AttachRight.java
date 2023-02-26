// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.clamps;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Clamps;

public class AttachRight extends CommandBase {
  private Clamps m_clamps;
  private Double m_speed;

  public AttachRight(Clamps clamps, Double speed) {
    m_clamps = clamps;
    m_speed = speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_clamps.attachRight(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_clamps.attachRight(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
