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
    addRequirements(m_arm);

    m_speed = speed;
    m_position = position;
  }

  @Override
  public void initialize() {
    m_arm.setDesiredTiltPosition(m_position);
  }

  @Override
  public void execute() {}
    
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getTiltEncoderPosition() - m_position) < 0.1;
  }

}
