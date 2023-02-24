// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/** Extends the scoring arm to a specified length. */
public class ExtendArmToLength extends CommandBase {
  private Arm m_arm;
  private Double m_speed;
  private Double m_position;

  public ExtendArmToLength(Arm arm, Double speed, Double position) {
    m_arm = arm;
    addRequirements(m_arm);

    m_speed = speed;
    m_position = position;
  }

  @Override
  public void initialize() {
    boolean isSafe = true; // TODO: m_arm.isSafeToExtend();
    if (isSafe) {
      m_arm.setDesiredExtensionPosition(m_position);
    } 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getExtensionEncoderPosition() - m_position) < 0.1;
  }
}
