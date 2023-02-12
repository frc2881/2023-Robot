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
    m_position = position;

    // Set speed depending on whether we are above or below the position wanted
      if(m_arm.getTiltEncoderPosition() < m_position){
        m_speed = speed;

      } else if(m_arm.getTiltEncoderPosition() > m_position){
        m_speed = -speed;
      }
      else{
        m_speed = 0.0;
      }

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // TODO: Add isSafe based on tilt position
    m_arm.runArm(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((m_speed < 0) && (m_arm.getExtensionEncoderPosition() <= m_position)) {
      return true;
    } else if((m_speed > 0) && (m_arm.getExtensionEncoderPosition() >= m_position)) {
      return true;
    } else {
      return false;
    }
  }
}
