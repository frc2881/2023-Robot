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
  private boolean m_isInfinite;

  public TiltArmToHeight(ArmTilt armTilt, double speed, double position, boolean isInfinite) {
    m_armTilt = armTilt;
    m_speed = speed;
    m_position = position;
    m_isInfinite = isInfinite;

    addRequirements(m_armTilt);
  }

  @Override
  public void initialize() {
    m_armTilt.setDesiredPosition(m_position, m_speed);
  }

  @Override
  public void execute(){}
    
  @Override
  public void end(boolean interrupted) {
    m_armTilt.run(0.0);
  }

  @Override
  public boolean isFinished() {
    if(m_isInfinite){
      return false;
    } else{
      return Math.abs(m_armTilt.getEncoderPosition() - m_position) < 0.1;
    }
    
  }

}
