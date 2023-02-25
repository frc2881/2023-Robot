// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmTilt;

/** Elevates the scoring arm. */
public class TiltArm extends CommandBase {
  private ArmTilt m_armTilt;
  private DoubleSupplier m_speed;
  
  public TiltArm(ArmTilt armTilt, DoubleSupplier speed) {
    m_armTilt = armTilt;
    m_speed = speed;

    addRequirements(m_armTilt);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_armTilt.run(-m_speed.getAsDouble());
    /*if(m_arm.isSafeToTilt()){
      m_arm.runTilt(-m_speed.getAsDouble());
    } else {
      m_arm.runTilt(0.0);
    }*/
  }

  @Override
  public void end(boolean interrupted) {
    m_armTilt.run(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
