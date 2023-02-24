// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;

public class ArmTiltOverride extends CommandBase {
  private Arm m_arm;

  public ArmTiltOverride(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_arm.enableTiltSoftLimits(false);
  }

  @Override
  public void execute() {
    m_arm.runTilt(-0.20);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.enableTiltSoftLimits(true);
    m_arm.resetTiltEncoder();
    m_arm.runTilt(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
