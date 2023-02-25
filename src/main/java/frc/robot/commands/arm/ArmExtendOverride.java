// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmExtension;

public class ArmExtendOverride extends CommandBase {
  private ArmExtension m_armExtension;

  public ArmExtendOverride(ArmExtension armExtension) {
    m_armExtension = armExtension;

    addRequirements(m_armExtension);
  }

  @Override
  public void initialize() {
    m_armExtension.enableSoftLimits(false);
  }

  @Override
  public void execute() {
    m_armExtension.run(-0.15);
  }

  @Override
  public void end(boolean interrupted) {
    m_armExtension.enableSoftLimits(true);
    m_armExtension.resetEncoder();
    m_armExtension.run(0.0);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
