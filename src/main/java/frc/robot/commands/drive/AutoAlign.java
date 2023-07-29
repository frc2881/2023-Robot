// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.Node;
import frc.robot.subsystems.Drive;

public class AutoAlign extends CommandBase {
  Drive m_drive;
  private final PIDController m_thetaController;
  private final PIDController m_yController;
  private final PIDController m_xController;
  Pose2d m_nearestNodePose;
  /** Creates a new AutoAlign. */
  public AutoAlign(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_thetaController = new PIDController(0.025, 0, 0.0015);
    m_thetaController.enableContinuousInput(-180.0, 180.0);
    m_thetaController.setTolerance(0.1, 0.5);
    m_thetaController.setSetpoint(180.0);

    m_yController = new PIDController(0.1, 0, 0);
    m_yController.setTolerance(0.1, 0.5);
    m_yController.setSetpoint(0.0);

    m_xController = new PIDController(0.01, 0, 0);

    addRequirements(m_drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nearestNodePose = m_drive.getNearestNode().pose;

    SmartDashboard.putNumber("Drive/yVel", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    Pose2d currentPose = m_drive.getPose();
    Rotation2d currentRotation = currentPose.getRotation();

    Transform2d delta = m_nearestNodePose.minus(currentPose);
    
    double rotationVel = m_thetaController.calculate(currentRotation.getDegrees());

    double pidMaxSpeed = 0.75;

    double yVel = MathUtil.clamp(
          m_yController.calculate((delta.getY() * 10)), 
          -pidMaxSpeed, 
          pidMaxSpeed);
    
    ;
 // INVERT - GOING THE WRONG WAY
    SmartDashboard.putNumberArray("Drive/Velocities", new double[] {yVel, rotationVel});
    SmartDashboard.putNumber("Drive/yVel", yVel);
    SmartDashboard.putNumberArray("Drive/Pose/Delta",  new double[] { delta.getX(), delta.getY(), delta.getRotation().getDegrees() });
    
    if(m_thetaController.atSetpoint()) {
      rotationVel = 0.0;
    }

    if(m_yController.atSetpoint()){
      yVel = 0.0;
    }

    if(Math.abs(delta.getX()) < 1 && Math.abs(delta.getY()) < 1){

      m_drive.setModuleStates(m_drive.convertToModuleStates(
            0.0,
            yVel,
            0.0)); //rotationVel)); 

    }

    
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
