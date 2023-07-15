// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.Node;
import frc.robot.subsystems.Drive;

public class AutoAlign extends CommandBase {
  Drive m_drive;
  private final PIDController m_thetaController;
  /** Creates a new AutoAlign. */
  public AutoAlign(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_thetaController = new PIDController(0.01, 0, 0);
    m_thetaController.enableContinuousInput(-180.0, 180.0);
    m_thetaController.setTolerance(0.5, 0.5);
    m_thetaController.setSetpoint(180.0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Node node = m_drive.getNearestNode();
    Pose2d currentPose = m_drive.getPose();
    Rotation2d currentRotation = currentPose.getRotation();

    double rotation = m_thetaController.calculate(currentRotation.getDegrees());

    if(m_thetaController.atSetpoint()) {
      rotation = 0.0;
    }

    m_drive.drive(0.0, //translationX * Constants.Drive.kMaxSpeedMetersPerSecond, 
          0.0, //translationY * Constants.Drive.kMaxSpeedMetersPerSecond
          rotation * Constants.Drive.kMaxAngularSpeed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
