// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmTiltOverride extends CommandBase {
  private Arm m_arm;
  /** Creates a new ArmTiltOverride. */
  public ArmTiltOverride(Arm arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_arm.enableTiltSoftLimits(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.tilt(-0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_arm.enableTiltSoftLimits(true);
    m_arm.resetTiltEncoder();

    m_arm.tilt(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
