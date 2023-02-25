// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTilt;

public class TiltArmToHeightWithSpeed extends CommandBase {
  private ArmTilt m_armTilt;
  private Double m_speed;
  private Double m_position;
  /** Creates a new ExtendArmToLengthWithSpeed. */
  public TiltArmToHeightWithSpeed(ArmTilt armTilt, Double speed, Double position) {
    m_armTilt = armTilt;
    m_speed = speed;
    m_position = position;

    addRequirements(m_armTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderPosition = m_armTilt.getEncoderPosition();
    double speed = 0.0;
      if(encoderPosition < m_position){
        speed = -m_speed;
      } 
      if(encoderPosition > m_position){
        speed = m_speed;
      }
      m_armTilt.run(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_armTilt.getEncoderPosition() - m_position) < 0.1){
      return true;
    } else{
      return false;
    }
    
  }
}
