// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmTilt;

public class ArmTiltOverride extends CommandBase {
  private ArmTilt m_armTilt;

  public ArmTiltOverride(ArmTilt armTilt) {
    m_armTilt = armTilt;

    addRequirements(m_armTilt);
  }

  @Override
  public void initialize() {
    m_armTilt.enableSoftLimits(false);
  }

  @Override
  public void execute() {
    m_armTilt.run(-0.10);
  }

  @Override
  public void end(boolean interrupted) {
    m_armTilt.enableSoftLimits(true);
    m_armTilt.resetEncoder();
    m_armTilt.run(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
