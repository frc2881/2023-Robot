// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final CANSparkMax m_extensionMotor;
  private final RelativeEncoder m_extensionMotorEncoder;
  private final CANSparkMax m_elevationMotor;
  private final RelativeEncoder m_elevationMotorEncoder;

  /** Creates a new Arm. */
  public Arm() {
    m_extensionMotor = new CANSparkMax(Constants.Arm.kExtensionMotorId, MotorType.kBrushless);
    // m_extensionMotor.setInverted(false); (Might need; test and see)
    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       (float)Constants.Arm.kExtendForwardLimit);
    m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       (float)Constants.Arm.kExtendReverseLimit);

    m_elevationMotor = new CANSparkMax(Constants.Arm.kElevationMotorId, MotorType.kBrushless);
    // m_elevationMotor.setInverted(false); (Might need; test and see)
    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       (float)Constants.Arm.kElevateForwardLimit);
    m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       (float)Constants.Arm.kElevateReverseLimit);

    m_elevationMotorEncoder = m_elevationMotor.getEncoder();
    m_elevationMotorEncoder.setPositionConversionFactor(Constants.Arm.kElevateRotationsToInches);

    m_extensionMotorEncoder = m_extensionMotor.getEncoder();
    m_extensionMotorEncoder.setPositionConversionFactor(Constants.Arm.kExtendRotationsToInches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Extends or retracts the arm
   * 
   * @param speed positive value extends
   */
  public void runArm(double speed) { 
    m_extensionMotor.set(speed);
  }

  /**
   * Elevates or lowers the arm.
   * 
   * @param speed positive value elevates.
   */
  public void elevate(double speed) { // TODO: Find better name?
    m_elevationMotor.set(speed);
  }

  public Double getExtensionEncoderPosition(){
    return m_extensionMotorEncoder.getPosition();
  }

  public Double getElevationEncoderPosition(){
    return m_elevationMotorEncoder.getPosition();
  }

}
