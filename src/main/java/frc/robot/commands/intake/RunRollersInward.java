// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** Runs the rollers forward.*/
public class RunRollersInward extends CommandBase {
  private Intake m_intake;
  
  public RunRollersInward(Intake intake) {
    m_intake = intake;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runRollersInward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
