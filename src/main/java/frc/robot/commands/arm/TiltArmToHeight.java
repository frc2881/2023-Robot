// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

/** Elevates the scoring arm to a specified height. */
public class TiltArmToHeight extends CommandBase {
  private Arm m_arm;
  private Intake m_intake;
  private Double m_speed;
  private Double m_position;

  public TiltArmToHeight(Arm arm, Intake intake, Double speed, Double position) {
    m_arm = arm;
    m_intake = intake;
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

    // addRequirements(m_arm);

  }

  @Override
  public void initialize() {
    System.out.println("Tilt Arm to height");
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

     m_arm.tilt(m_speed);
  }
    

  @Override
  public void end(boolean interrupted) {
    m_arm.tilt(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if((m_speed < 0) && (m_arm.getTiltEncoderPosition() <= m_position)){
        return true;
      } else if((m_speed > 0) && (m_arm.getTiltEncoderPosition() >= m_position)) {
        return true;
      } else {
        return false;
      }
    }

}
