// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final CANSparkMax m_extensionMotor;

  /** Creates a new Arm. */
  public Arm() {
    m_extensionMotor = new CANSparkMax(Constants.Arm.kExtensionMotorId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend(double speed) {
    m_extensionMotor.set(speed);
  }

  public void retract(double speed) {
    m_extensionMotor.set(-speed);
  }
}
