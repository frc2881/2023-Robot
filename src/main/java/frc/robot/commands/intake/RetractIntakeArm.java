// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class RetractIntakeArm extends CommandBase {
  private Intake m_intake;
  
  public RetractIntakeArm(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.retract(); 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return true;
  }
}
