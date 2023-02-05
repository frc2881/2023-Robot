// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RetractIntakeArm extends CommandBase {
  private Intake m_intake;
  /** Creates a new RetractIntakeArm. */
  public RetractIntakeArm(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.moveArm(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.moveArm(0.0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
