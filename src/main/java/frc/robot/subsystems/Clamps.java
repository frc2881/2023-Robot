// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Clamps extends SubsystemBase {
  private final CANSparkMax m_left;
  private final CANSparkMax m_right;
 
  public Clamps() {
    m_left = new CANSparkMax(12, MotorType.kBrushless);
    m_left.setIdleMode(IdleMode.kBrake);

    m_right = new CANSparkMax(13, MotorType.kBrushless);
    m_right.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public void updateTelemetry() {}
}
