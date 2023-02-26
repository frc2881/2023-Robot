// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTilt;

public class TiltArmToHeight extends CommandBase {
  private ArmTilt m_armTilt;
  private Double m_speed;
  private Double m_position;

  public TiltArmToHeight(ArmTilt armTilt, Double speed, Double position) {
    m_armTilt = armTilt;
    m_speed = speed;
    m_position = position;

    addRequirements(m_armTilt);
  }

  @Override
  public void initialize() {
    m_armTilt.setDesiredPosition(m_position, m_speed);
  }

  @Override
  public void execute() {
    /* 
    boolean intakeIsOut = m_intake.isOut;
    boolean isSafe = m_arm.isSafeToTilt();
    
    if(m_speed < 0){
      if(intakeIsOut || isSafe == false){
        m_arm.tilt(0.0);
      } else {
        m_arm.tilt(m_speed);
      }
    }*/
  }
    
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_armTilt.getEncoderPosition() - m_position) < 0.1;
  }

}
