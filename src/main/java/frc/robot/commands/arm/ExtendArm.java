// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmExtension;

public class ExtendArm extends CommandBase {
  private ArmExtension m_armExtension;
  private DoubleSupplier m_speed;

  public ExtendArm(ArmExtension armExtension, DoubleSupplier speed) {
    m_armExtension = armExtension;
    m_speed = speed;

    addRequirements(m_armExtension);    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_armExtension.run(-m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armExtension.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
