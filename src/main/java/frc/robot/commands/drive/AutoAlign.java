// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutoAlign extends CommandBase {
  Drive m_drive;

  private final PIDController m_thetaController;
  private final PIDController m_yController;
  private final PIDController m_xController;

  Pose2d m_nearestNodePose;

  public AutoAlign(Drive drive) {
    m_drive = drive;

    m_thetaController = new PIDController(0.100, 0, 0.01);
    m_thetaController.enableContinuousInput(-180.0, 180.0);
    m_thetaController.setTolerance(0.5);
    m_thetaController.setSetpoint(180.0);

    m_yController = new PIDController(0.6, 0, 0.06);
    m_yController.setTolerance(0.25);
    m_yController.setSetpoint(0.0);

    m_xController = new PIDController(0.3, 0, .03);
    m_xController.setTolerance(0.25);
    m_xController.setSetpoint(0.0);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nearestNodePose = m_drive.getNearestNode().pose;
    m_drive.setIsAutoAlignStarted(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = m_drive.getPose();
    Rotation2d currentRotation = currentPose.getRotation();

    Transform2d delta = m_nearestNodePose.minus(currentPose);
    SmartDashboard.putNumberArray("Drive/Pose/Delta",  new double[] { delta.getX(), delta.getY(), delta.getRotation().getDegrees() });
    
    double rotationVel = m_thetaController.calculate(currentRotation.getDegrees());
    rotationVel += Math.copySign(0.15, rotationVel); 

    double pidMaxSpeed = 1.0;

    double yVel = MathUtil.clamp(
          m_yController.calculate((delta.getY() * 10)), 
          -pidMaxSpeed, 
          pidMaxSpeed);

    double xVel = MathUtil.clamp(
          m_xController.calculate((delta.getX() * 10)), 
          -pidMaxSpeed, 
          pidMaxSpeed);
    
    if (m_thetaController.atSetpoint()) {
      rotationVel = 0.0;
    }

    if (m_yController.atSetpoint()){
      yVel = 0.0;
    }

    if (m_xController.atSetpoint()){
      xVel = 0.0;
    }

    m_drive.setModuleStates(m_drive.convertToModuleStates(
          -xVel,
          -yVel, 
          rotationVel));

    if (rotationVel <= 0.1 && xVel <= 0.1 && yVel <= 0.1) {
      m_drive.setIsAutoAlignCompleted(true);
    } else {
      m_drive.setIsAutoAlignCompleted(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setIsAutoAlignStarted(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
