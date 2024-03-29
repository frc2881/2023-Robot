// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtension;

/** Extends the scoring arm to a specified length. */
public class ExtendArmToLength extends CommandBase {
  private ArmExtension m_armExtension;
  private double m_speed;
  private double m_position;

  public ExtendArmToLength(ArmExtension armExtension, double speed, double position) {
    m_armExtension = armExtension;
    m_speed = speed;
    m_position = position;

    addRequirements(m_armExtension);
  }

  @Override
  public void initialize() {
    m_armExtension.setDesiredPosition(m_position, m_speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_armExtension.run(0.0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_armExtension.getEncoderPosition() - m_position) < 0.1;
  }
}
