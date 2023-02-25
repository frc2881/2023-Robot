// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;

/** Elevates the scoring arm. */
public class TiltArm extends CommandBase {
  private Arm m_arm;
  private DoubleSupplier m_speed;
  
  public TiltArm(Arm arm, DoubleSupplier speed) {
    m_arm = arm;
    m_speed = speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_arm.runTilt(-m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.runTilt(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
