// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/** Extends the scoring arm. */
public class ExtendArm extends CommandBase {
  private Arm m_arm;
  private DoubleSupplier m_speed;

  public ExtendArm(Arm arm, DoubleSupplier speed) {
    m_arm = arm;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean isSafe = m_arm.isSafeToExtend();
    if(isSafe == true){
      m_arm.runExtension(m_speed.getAsDouble());
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runExtension(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
