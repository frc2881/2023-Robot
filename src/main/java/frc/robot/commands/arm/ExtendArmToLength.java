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

  }

  @Override
  public void initialize() {
    System.out.println("Extend Arm to Length");
    boolean isSafe = m_arm.isSafeToExtend();
    if(isSafe == true){
      m_arm.setDesiredExtensionPosition(m_position);
    }
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_arm.getExtensionEncoderPosition() - m_position) < 0.1) {
      return true;
    } else {
      return false;
    }
  }
}
