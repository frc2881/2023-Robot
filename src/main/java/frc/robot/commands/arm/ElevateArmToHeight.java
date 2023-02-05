// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/** Elevates the scoring arm to a specified height. */
public class ElevateArmToHeight extends CommandBase {
  private Arm m_arm;
  private Double m_speed;
  private Double m_position;

  public ElevateArmToHeight(Arm arm, Double speed, Double position) {
    m_arm = arm;
    m_speed = speed;
    m_position = position;

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_arm.elevate(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.elevate(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((m_speed < 0) && (m_arm.getElevationEncoderPosition() <= m_position)) {
      return true;
    } else if((m_speed > 0) && (m_arm.getElevationEncoderPosition() >= m_position)) {
      return true;
    } else {
      return false;
    }
  }
}
