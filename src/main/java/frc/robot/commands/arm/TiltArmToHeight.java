// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class TiltArmToHeight extends CommandBase {
  private Arm m_arm;
  private Intake m_intake;
  private Double m_speed;
  private Double m_position;

  public TiltArmToHeight(Arm arm, Intake intake, Double speed, Double position) {
    m_arm = arm;
    m_intake = intake;
    m_position = position;

  }

  @Override
  public void initialize() {
    System.out.println("Tilt Arm to height");
    m_arm.setDesiredTiltPosition(m_position);
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(Math.abs(m_arm.getTiltEncoderPosition() - m_position) < 0.1){
        return true;
      } else {
        return false;
      }
    }

}
