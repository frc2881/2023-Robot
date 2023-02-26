// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Clamps extends SubsystemBase {
  private final CANSparkMax m_left;
  private final CANSparkMax m_right;
 
  public Clamps() {
    m_left = new CANSparkMax(Constants.Clamps.kLeftClampCANId, MotorType.kBrushless);
    m_left.setIdleMode(IdleMode.kBrake);
    m_left.setSmartCurrentLimit(Constants.Clamps.kCurrentLimit);
    
    m_left.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_left.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       Constants.Clamps.kReverseSoftLimit); 

    m_right = new CANSparkMax(Constants.Clamps.kRightClampCANId, MotorType.kBrushless);
    m_right.setIdleMode(IdleMode.kBrake);
    m_right.setSmartCurrentLimit(Constants.Clamps.kCurrentLimit);
    
    /*m_right.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_right.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       Constants.Clamps.kReverseSoftLimit); */
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public void attachLeft(double speed) {
    m_left.set(speed);
  }
  public void attachRight(double speed) {
    m_right.set(speed);
  }

  public void updateTelemetry() {}
}
